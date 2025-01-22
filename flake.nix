{
  description = "Implementation of a parser for hpp-manipulation";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/refs/pull/362956/head";
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    # requires https://github.com/humanoid-path-planner/hpp-manipulation/pull/191
    hpp-manipulation = {
      url = "github:florent-lamiraux/hpp-manipulation/devel";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-parts.follows = "flake-parts";
    };
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      systems = [
        "x86_64-linux"
        "aarch64-linux"
        "aarch64-darwin"
        "x86_64-darwin"
      ];
      perSystem =
        { pkgs, self', system, ... }:
        {
          devShells.default = pkgs.mkShell {
            inputsFrom = [ self'.packages.default ];
            ROS_PACKAGE_PATH = "${pkgs.example-robot-data}/share";
          };
          packages = {
            default = self'.packages.hpp-manipulation-urdf;
            hpp-manipulation-urdf = (pkgs.hpp-manipulation-urdf.override {
              inherit (inputs.hpp-manipulation.packages.${system}) hpp-manipulation;
            }).overrideAttrs (_: {
              # TODO: remove this after next release
              patches = [];
              src = pkgs.lib.fileset.toSource {
                root = ./.;
                fileset = pkgs.lib.fileset.unions [
                  ./CMakeLists.txt
                  ./doc
                  ./include
                  ./package.xml
                  ./src
                  ./tests
                ];
              };
            });
          };
        };
    };
}
