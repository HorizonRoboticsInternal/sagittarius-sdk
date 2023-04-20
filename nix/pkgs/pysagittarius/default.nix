{ lib
, fetchFromGitHub
, buildPythonPackage
, pythonOlder
, cmake
, pybind11
, numpy
, eigen
, boost
}:

buildPythonPackage rec {
  pname = "pysagittarius";
  version = "1.0.0";
  format = "setuptools";  

  disabled = pythonOlder "3.8";
  
  src = ../../../.;

  propagatedBuildInputs = [
    numpy
  ];

  buildInputs = [
    pybind11
    eigen
    boost
  ];

  nativeBuildInputs = [
    cmake
  ];

  dontUseCmakeConfigure = true;  

  pythonImportsCheck = [ "pysagittarius" ];

  meta = with lib; {
    homepage = "https://github.com/HorizonRoboticsInternal/sagittarius-sdk";
    description = ''
      Python wrapper of SDK for the K1 (Sagittarius) robotic arm
    '';
    platforms = with platforms; linux ++ darwin;
    licencse = licenses.mit;
  };
}
