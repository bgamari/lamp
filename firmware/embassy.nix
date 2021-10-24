{ pkgs }:

with pkgs;
let
  click = { buildPythonPackage, fetchPypi }: buildPythonPackage rec {
    pname = "click";
    version = "8.0.1";
    src = fetchPypi {
      inherit pname version;
      sha256 = "sha256:0ymdyf37acq4qxh038q0xx44qgj6y2kf0jd0ivvix6qij88w214c";
    };
  };

  svdtools = { buildPythonPackage, fetchPypi, click, lxml, pyyaml }: buildPythonPackage rec {
    pname = "svdtools";
    version = "0.1.15";
    src = fetchPypi {
      inherit pname version;
      sha256 = "sha256:0l84yl040jrz9qfvzpjlq18if8rg7fcr7wrdncbjx6qnsymzf6rb";
    };
    propagatedBuildInputs = [ click lxml pyyaml ];
  };

  pyPkgs = python3Packages.override {
    overrides = self: super: {
      click = self.callPackage click {};
      svdtools = self.callPackage svdtools {};
    };
  };

  pyEnv = pyPkgs.python.withPackages (pkgs: with pkgs; [
    xmltodict pyyaml toml
  ]);
in symlinkJoin {
  name = "env";
  paths = [
    pyEnv
    pyPkgs.svdtools
    pyPkgs.libxml2.bin # for xmllint
  ];
}

