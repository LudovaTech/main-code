{
  inputs = {
    rust-overlay.url = "github:oxalica/rust-overlay/stable";
    cargo2nix = {
      url = "github:cargo2nix/cargo2nix/release-0.12";
      inputs.rust-overlay.follows = "rust-overlay";
    };
    flake-utils.follows = "cargo2nix/flake-utils";
    nixpkgs.follows = "cargo2nix/nixpkgs";
  };

  outputs = inputs: with inputs;
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          overlays = [cargo2nix.overlays.default];
        };

        rustPkgs = pkgs.rustBuilder.makePackageSet {
          rustVersion = "1.88.0";
          packageFun = import ./Cargo.nix;
        };

      in rec {
        packages = {
          # replace hello-world with your package name
          camera = (rustPkgs.workspace.camera {});
          default = packages.camera;
        };
      }
    );
}