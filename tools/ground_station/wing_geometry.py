"""Equal-span, tip-connected wings in a rear view; positive roll lowers right tip."""
import math

ORDERS = {1: ("A",), 3: ("B", "A", "C"), 4: ("D", "B", "A", "C"),
          5: ("D", "B", "A", "C", "E"), 7: ("F", "D", "B", "A", "C", "E", "G")}


def wing_segments(configuration):
    """Return (name, left endpoint, right endpoint, absolute roll) in span units."""
    count = configuration["count"]
    order = ORDERS[count]
    base = configuration["roll_a"]
    angles = {"A": base, "B": base + configuration["ab"], "C": base + configuration["ac"],
              "D": base + configuration["ab"] + configuration["bd"],
              "E": base + configuration["ac"] + configuration["ce"],
              "F": base + configuration["ab"] + configuration["bd"] + configuration["df"],
              "G": base + configuration["ac"] + configuration["ce"] + configuration["eg"]}
    def vector(name):
        radians = math.radians(angles[name])
        return math.cos(radians), math.sin(radians)
    ax, ay = vector("A")
    segments = {"A": ((-ax / 2, -ay / 2), (ax / 2, ay / 2))}
    tip = segments["A"][0]
    for name in reversed(order[:order.index("A")]):
        dx, dy = vector(name)
        other = tip[0] - dx, tip[1] - dy
        segments[name] = other, tip
        tip = other
    tip = segments["A"][1]
    for name in order[order.index("A")+1:]:
        dx, dy = vector(name)
        other = tip[0] + dx, tip[1] + dy
        segments[name] = tip, other
        tip = other
    return [(name, *segments[name], angles[name]) for name in order]


def configuration_ready(configuration):
    return bool(configuration and configuration.get("config_valid", 1) == 1 and configuration["master"] == 1 and
                (configuration["count"] == 1 or
                 configuration["left_valid"] == 1 and configuration["right_valid"] == 1))


def aircraft_attitudes(configuration, data, local_name=None):
    """Missing attitude stays None; remote yaw is not in the sensor protocol."""
    result = {}
    if configuration:
        for name in ORDERS[configuration["count"]]:
            key = name.lower()
            valid = configuration.get(f"v_{key}") == 1
            result[name] = {"roll": configuration.get(f"r_{key}") if valid else None,
                            "pitch": configuration.get(f"p_{key}") if valid else None,
                            "yaw": None}
    if data and local_name:
        result[local_name] = {key: data.get(key) for key in ("roll", "pitch", "yaw")}
    return result


def wing_surfaces(configuration):
    """Equal-span, tip-connected wing panels rotated by each IMU roll/pitch.

    Coordinates are (right, forward, up). Heading is shared because remote
    yaw is not transmitted; this is an attitude schematic, not a position map.
    """
    order = ORDERS[configuration["count"]]
    attitudes = aircraft_attitudes(configuration, {})
    if not configuration_ready(configuration) or any(
            attitudes[name][key] is None for name in order for key in ("roll", "pitch")):
        return []
    def vectors(name):
        roll, pitch = (math.radians(attitudes[name][key]) for key in ("roll", "pitch"))
        return ((math.cos(roll), math.sin(pitch) * math.sin(roll), -math.cos(pitch) * math.sin(roll)),
                (0, math.cos(pitch), math.sin(pitch)))
    def add(a, b, factor=1):
        return tuple(x + factor * y for x, y in zip(a, b))
    span, _ = vectors("A")
    tips = {"A": (tuple(-v / 2 for v in span), tuple(v / 2 for v in span))}
    tip = tips["A"][0]
    for name in reversed(order[:order.index("A")]):
        span, _ = vectors(name)
        tips[name] = add(tip, span, -1), tip
        tip = tips[name][0]
    tip = tips["A"][1]
    for name in order[order.index("A") + 1:]:
        span, _ = vectors(name)
        tips[name] = tip, add(tip, span)
        tip = tips[name][1]
    surfaces = []
    for name in order:
        left, right = tips[name]
        _, chord = vectors(name)
        center = tuple((x + y) / 2 for x, y in zip(left, right))
        corners = (add(left, chord, -.25), add(right, chord, -.25),
                   add(right, chord, .25), add(left, chord, .25))
        surfaces.append({"name": name, "left": left, "right": right, "corners": corners,
                         "center": center, "nose": add(center, chord, .45), **attitudes[name]})
    return surfaces


def project_wing(point):
    """Oblique view keeps pitch visible even when roll is zero."""
    right, forward, up = point
    return right + .4 * forward, -up - .65 * forward
