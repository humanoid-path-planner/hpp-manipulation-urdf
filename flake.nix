{
  description = "Implementation of a parser for hpp-manipulation";

  inputs = {
    gepetto.url = "github:gepetto/nix";
    flake-parts.follows = "gepetto/flake-parts";
    nixpkgs.follows = "gepetto/nixpkgs";
    nix-ros-overlay.follows = "gepetto/nix-ros-overlay";
    systems.follows = "gepetto/systems";
    treefmt-nix.follows = "gepetto/treefmt-nix";
  };

  outputs =
    inputs:
    inputs.flake-parts.lib.mkFlake { inherit inputs; } (
      { lib, self, ... }:
      {
        systems = import inputs.systems;
        imports = [
          inputs.gepetto.flakeModule
          { gepetto-pkgs.overlays = [ self.overlays.default ]; }
        ];
        flake.overlays.default = _final: prev: {
          hpp-manipulation-urdf = prev.hpp-manipulation-urdf.overrideAttrs {
            src = lib.fileset.toSource {
              root = ./.;
              fileset = lib.fileset.unions [
                ./CMakeLists.txt
                ./doc
                ./include
                ./package.xml
                ./src
                ./tests
              ];
            };
          };
        };
        perSystem =
          { pkgs, self', ... }:
          {
            packages = {
              default = self'.packages.hpp-manipulation-urdf;
              hpp-manipulation-urdf = pkgs.hpp-manipulation-urdf;
            };
          };
      }
    );
}
