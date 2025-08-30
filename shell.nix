# Easy way to enable rust support in NixOS

{  pkgs ? import <nixpkgs> {} }:

pkgs.mkShellNoCC {

  packages = with pkgs; [
    rustup
  ];

  shellHook = ''
    rustup toolchain install stable
  '';
}