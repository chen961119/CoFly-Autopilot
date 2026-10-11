import struct
import unittest
import zlib
from dataclasses import replace
from parameter_file import FILE_VERSION, CrcMismatchError, encode_sd_file, decode_sd_file
from protocol import Parameter


class ParameterFileTests(unittest.TestCase):
    def setUp(self):
        self.parameters = {
            "gain": Parameter("gain", .25, -10, 10, "T", "T"),
            "pwm_rev": Parameter("pwm_rev", -1, -1, 1, "T", "T", "int"),
            "aircraft_id": Parameter("aircraft_id", 2, 0, 7, "A", "A", "int", "EEPROM", "REBOOT"),
        }

    def file(self, records, count=None):
        body = (FILE_VERSION + "\nCOUNT=" + str(len(records) if count is None else count) + "\n" + "".join(records)).encode("ascii")
        return body + f"CRC32={zlib.crc32(body):08x}\n".encode("ascii")

    def test_exact_text_and_eeprom_exclusion(self):
        expected = self.file(["gain:float=0.25\n", "pwm_rev:int=-1\n"])
        self.assertEqual(encode_sd_file(self.parameters), expected)
        self.assertNotIn(b"aircraft_id", expected)
        self.assertEqual(decode_sd_file(expected, self.parameters), {"gain": .25, "pwm_rev": -1})

    def test_names_have_variable_length_and_configured_limit(self):
        for width in (1, 27, 48, 128):
            p = Parameter("x" * width, 1, 0, 10, "T", "T")
            data = encode_sd_file({p.name: p}, width)
            self.assertIn((p.name + ":float=1\n").encode(), data)
            self.assertNotIn(b"\0", data)
            self.assertEqual(decode_sd_file(data, {p.name: p}), {p.name: 1})
            with self.assertRaises(ValueError):
                encode_sd_file({p.name + "x": replace(p, name=p.name + "x")}, width)

    def test_crc_failure_and_explicit_manual_edit_override(self):
        edited = encode_sd_file(self.parameters).replace(b"gain:float=0.25", b"gain:float=0.5")
        with self.assertRaises(CrcMismatchError):
            decode_sd_file(edited, self.parameters)
        self.assertEqual(decode_sd_file(edited, self.parameters, allow_crc_mismatch=True)["gain"], .5)

    def test_crc_override_cannot_bypass_invalid_values_or_schema(self):
        for record in ("gain:float=11\n", "gain:int=1\n", "unknown:float=1\n", "gain:float=nan\n", "gain:float=1e-100\n"):
            data = self.file([record, "pwm_rev:int=-1\n"])
            with self.assertRaises(ValueError):
                decode_sd_file(data, self.parameters, allow_crc_mismatch=True)

    def test_invalid_container_rejected(self):
        data = encode_sd_file(self.parameters)
        for invalid in (b"", data[:-1], data + b"extra\n", data.replace(b"\n", b"\r\n"),
                        b"\xef\xbb\xbf" + data, data.replace(b"V8", b"V7"),
                        data.replace(b"CRC32=", b"CRC32=x"), data.replace(b"gain", b"gai\0"),
                        self.file(["gain:float=0.25\n", "pwm_rev:int=-1\n"], 3)):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    decode_sd_file(invalid, self.parameters, allow_crc_mismatch=True)

    def test_duplicate_unknown_eeprom_missing_and_wrong_type_rejected(self):
        for records in (["gain:float=0.25\n", "gain:float=0.5\n"],
                        ["gain:float=0.25\n", "aircraft_id:int=2\n"],
                        ["gain:float=0.25\n"], ["gain:int=1\n", "pwm_rev:int=-1\n"],
                        ["unknown:float=1\n", "pwm_rev:int=-1\n"]):
            with self.assertRaises(ValueError):
                decode_sd_file(self.file(records), self.parameters)

    def test_numeric_formats_ranges_and_overflows(self):
        for text in ("nan", "inf", "1e100", "1e-100", "1e-1000", "0.5x", " 0.5", "0.5 ", "0" * 64, "11"):
            with self.subTest(text=text):
                with self.assertRaises(ValueError):
                    decode_sd_file(self.file([f"gain:float={text}\n", "pwm_rev:int=-1\n"]), self.parameters)
        for text in ("0", "1.0", "1e0", "2147483648", "-2147483649"):
            with self.assertRaises(ValueError):
                decode_sd_file(self.file(["gain:float=0.25\n", f"pwm_rev:int={text}\n"]), self.parameters)
        self.assertEqual(decode_sd_file(self.file(["gain:float=5e-1\n", "pwm_rev:int=-1\n"]), self.parameters)["gain"], .5)

    def test_float32_extremes_round_trip_bits(self):
        for bits in (0, 0x80000000, 1, 0x80000001, 0x00800000, 0x7f7fffff, 0xff7fffff, 0x3eaaaaab):
            value = struct.unpack("<f", struct.pack("<I", bits))[0]
            p = Parameter("value", value, -3.4028234663852886e38, 3.4028234663852886e38, "T", "T")
            data = encode_sd_file({p.name: p})
            restored = decode_sd_file(data, {p.name: p})[p.name]
            self.assertEqual(struct.pack("<f", restored), struct.pack("<I", bits))

    def test_int32_extremes_round_trip(self):
        for value in (-2147483648, 2147483647):
            p = Parameter("value", value, -2147483648, 2147483647, "T", "T", "int")
            self.assertEqual(decode_sd_file(encode_sd_file({p.name: p}), {p.name: p})[p.name], value)

    def test_readonly_values_cannot_be_changed(self):
        p = replace(self.parameters["gain"], read_only=True)
        params = {"gain": p, "pwm_rev": self.parameters["pwm_rev"]}
        self.assertEqual(decode_sd_file(encode_sd_file(params), params)["gain"], .25)
        with self.assertRaises(ValueError):
            decode_sd_file(self.file(["gain:float=0.5\n", "pwm_rev:int=-1\n"]), params)

    def test_outbound_invalid_names_types_duplicates_and_limits(self):
        for p in (replace(self.parameters["gain"], name="bad-name"), replace(self.parameters["gain"], dtype="bad")):
            with self.assertRaises(ValueError):
                encode_sd_file({"p": p})
        with self.assertRaises(ValueError):
            encode_sd_file({"a": self.parameters["gain"], "b": self.parameters["gain"]})
        with self.assertRaises(ValueError):
            encode_sd_file({"ee": self.parameters["aircraft_id"]})
        with self.assertRaises(ValueError):
            encode_sd_file(self.parameters, 0)
