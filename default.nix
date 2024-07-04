{
  lib,
  stdenv,
  cmake,
  example-robot-data,
  hpp-manipulation,
}:

stdenv.mkDerivation {
  pname = "hpp-manipulation-urdf";
  version = "5.0.0";

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

  strictDeps = true;

  nativeBuildInputs = [ cmake ];
  propagatedBuildInputs = [ hpp-manipulation ];
  doCheck = true;

  preCheck = ''
    export ROS_PACKAGE_PATH=${example-robot-data}/share
  '';

  meta = {
    description = "Implementation of a parser for hpp-manipulation";
    homepage = "https://github.com/humanoid-path-planner/hpp-manipulation-urdf";
    license = lib.licenses.bsd2;
    maintainers = [ lib.maintainers.nim65s ];
  };
}
