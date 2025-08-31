# Easy way to enable rust support in NixOS

{  pkgs ? import <nixpkgs> {} }:

pkgs.mkShellNoCC {

  packages = with pkgs; [
    rustup
    podman # to cross compile to arm64
  ];

  LIBCLANG_PATH="${pkgs.llvmPackages.libclang.lib}/lib";

  shellHook = ''
    rustup toolchain install stable
    cargo install cross --git https://github.com/cross-rs/cross # to cross compile to arm64
    export PATH=$PATH:~/.cargo/bin/ # add cargo applications (cross, cross-util...) to path
  '';
}
