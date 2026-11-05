# Picard

Picard is a software that allows to connect hardware instrument panels with
Microsoft Flight Simulator using SimConnect. The purpose of Picard is to provide
a seamless integration between physical instrument panels and the flight
simulator, allowing pilots to have a more realistic and immersive experience.
The software is written in Rust and uses a multithreaded architecture to mediate
between the simulator and multiple hardware panels using serial connections.

## Building

Picard runs on Windows and can be built with `cargo build --release`.

It can also be cross-compiled from Linux using MinGW-w64.
This requires:

- the Windows target (`rustup target add x86_64-pc-windows-gnu`)
- the MinGW-w64 GCC with its static winpthreads library and libclang:
  - Debian: `gcc-mingw-w64-x86-64` and `libclang-dev`.
  - Fedora: `mingw64-gcc`, `mingw64-winpthreads-static` and `clang-devel`.

```bash
cargo build --release --target x86_64-pc-windows-gnu
```

Picard needs `SimConnect.dll` next to the executable and reads `config.toml`
from the working directory. The build places the DLL next to the generated
bindings in `target/x86_64-pc-windows-gnu/release/build/simconnect-sdk-*/out/`.
