{ pkgs ? import <nixpkgs> {} }:

let
  # OCP (OpenCASCADE Python bindings, used by CadQuery) ships precompiled .so
  # files that dlopen libstdc++, libGL, fontconfig, etc. Surface them through
  # LD_LIBRARY_PATH so `import cadquery` works inside the nix-shell.
  ocpRuntimeLibs = with pkgs; [
    stdenv.cc.cc.lib
    libglvnd
    libGLU
    fontconfig.lib
    freetype
    xorg.libXi
    xorg.libSM
    xorg.libICE
    xorg.libX11
    xorg.libXext
    xorg.libXrender
    expat
    zlib
    # Vulkan loader + Mesa's software Vulkan ICD (Lavapipe). This lets
    # wgpu/pygfx pick a working WebGPU backend on any Linux box —
    # including CI runners with no GPU — without needing libGL.so.
    vulkan-loader
    mesa             # provides Lavapipe (lvp_icd.x86_64.json)
  ];
in
pkgs.mkShell {
  name = "mechproof-dev-shell";

  # Packages to install in the environment
  buildInputs = with pkgs; [
    verilator
    iverilog      # Icarus Verilog for quick simulations
    yosys
    pkg-config
    cmake
    elan
    (python3.withPackages (ps: with ps; [numpy matplotlib pyyaml pandas pip jupytext nbconvert jupyterlab]))
    nodejs
    pkgsCross.riscv32-embedded.buildPackages.gcc
    pkgsCross.riscv32-embedded.buildPackages.binutils
    # Linux kernel cross-toolchain (glibc target). Needed to build the
    # in-tree sparkle-bitnet driver and the rv32 Linux kernel image.
    pkgsCross.riscv64.buildPackages.gcc
    pkgsCross.riscv64.buildPackages.binutils
    # Device-tree compiler — turns sparkle-soc.dts into the .dtb that
    # OpenSBI hands to Linux at boot.
    dtc
    # Kernel build prerequisites
    bc flex bison openssl
    cpio gzip
    nlohmann_json
    libuuid
    zstd.dev
  ] ++ ocpRuntimeLibs;

  # Environment variables
  shellHook = ''
    echo "--- MechProof Development Environment ---"
    echo "Verilator version: $(verilator --version)"
    echo "-----------------------------------------"

    # Set VERILATOR_ROOT if your build system needs it
    export VERILATOR_ROOT=${pkgs.verilator}/share/verilator

    # Runtime libs for OCP (CadQuery). The OCP wheel dlopens these at import.
    export LD_LIBRARY_PATH=${pkgs.lib.makeLibraryPath ocpRuntimeLibs}''${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}

    # Point the Vulkan loader at Mesa's Lavapipe (software) ICD so wgpu
    # / pygfx can grab a Vulkan device without needing real GPU drivers.
    export VK_ICD_FILENAMES=${pkgs.mesa}/share/vulkan/icd.d/lvp_icd.x86_64.json
  '';
}
