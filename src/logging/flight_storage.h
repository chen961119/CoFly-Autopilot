#pragma once

// Runtime callers are the single background parameter/GCS task. This waits for
// the storage task and preserves the lifetime of the request and its buffers.
// Before startFlightStorage(), boot code executes the operation directly.
using StorageOperation = bool (*)(void *);
bool runStorageOperation(StorageOperation operation, void *context);
