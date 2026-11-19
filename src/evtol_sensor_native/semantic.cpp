#include <cstdint>
#include <cstddef>
#ifdef _WIN32
#define EVTOL_EXPORT __declspec(dllexport)
#else
#define EVTOL_EXPORT __attribute__((visibility("default")))
#endif

// Input and output are owned by the caller; no global state or Python ABI.
// The 24-bit lookup validates all three BGR channels exactly.
extern "C" EVTOL_EXPORT void evtol_decode_bgr(
    const uint8_t* source, uint8_t* destination, const uint8_t* palette,
    size_t pixels) {
    for (size_t i=0; i<pixels; ++i) {
        const uint32_t index=(uint32_t(source[0])<<16)|(uint32_t(source[1])<<8)|source[2];
        destination[i]=palette[index];
        source+=3;
    }
}
