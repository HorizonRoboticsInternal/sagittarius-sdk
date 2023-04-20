{ lib
, stdenv
, overrideCC
, gcc12
, cmake
, eigen
, boost
, buildDaemon ? false
, buildPython ? false
}:

(overrideCC stdenv gcc12).mkDerivation rec {
  pname = "sagittarius-sdk";
  version = "1.0.0";

  src = ../../../.;

  nativeBuildInputs = [ cmake ];

  buildInputs = [
    eigen
    boost
  ];

  cmakeFlags = [
    "-DINCLUDE_EXAMPLES=ON"
    (if buildDaemon then "-DBUILD_DAEMON=ON" else "-DBUILD_DAEMON=OFF")
    (if buildPython then "-DBUILD_PYTHON=ON" else "-DBUILD_PYTHON=OFF")
  ];

  meta = with lib; {
    homepage = "https://github.com/HorizonRoboticsInternal/sagittarius-sdk";
    description = ''
      SDK for the K1 (Sagittarius) robotic arm
    '';
    platforms = with platforms; linux ++ darwin;
    licencse = licenses.mit;
  };
}
