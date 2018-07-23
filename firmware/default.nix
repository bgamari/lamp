let nixpkgs = import ~/.nix-overlay/nixpkgs {};
in with nixpkgs; stdenv.mkDerivation {
  name = "firmware";
  nativeBuildInputs = [ gcc-arm-embedded gnumake openocd ];
}
