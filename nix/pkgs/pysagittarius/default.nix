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
  };
}
