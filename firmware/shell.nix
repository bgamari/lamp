let 
  nixpkgs = 
    let src = fetchGit {
      url = "https://github.com/nixos/nixpkgs";
      rev = "15379213a4bd7f91605418f0b9d74653ee4cea10";
      ref = "release-21.05";
    };
    in import src { overlays = [ (import rustOverlay) ]; };

  rustOverlay = fetchGit {
    url = "https://github.com/mozilla/nixpkgs-mozilla";
    rev = "4a07484cf0e49047f82d83fd119acffbad3b235f";
  };
in with nixpkgs;

let
  rustChannel = rustChannelOfTargets
    "nightly"
    "2021-12-27"
    [
      "x86_64-unknown-linux-gnu"
      "thumbv6m-none-eabi"
      "thumbv7m-none-eabi"
      "thumbv7em-none-eabi"
      "thumbv7em-none-eabihf"
      "wasm32-unknown-unknown"
    ];

in stdenv.mkDerivation {
  name = "rust";
  buildInputs = 
    let
    in [ rustChannel gcc-arm-embedded (import ./embassy.nix {pkgs = nixpkgs;}) ] ++
         (with nixpkgs; [ pkgconfig zlib libusb openocd ]);
  TARGET_CC = "${nixpkgs.gcc-arm-embedded}/bin/arm-none-eabi-gcc";
  TARGET = "thumbv6m-none-eabi";
  LD_LIBRARY_PATH = with nixpkgs; [ "${zlib}/lib" "${libusb}/lib" ];
  PKG_CONFIG_PATH = with nixpkgs; [ "${zlib}/lib/pkgconfig" "${libusb.dev}/lib/pkgconfig" ];
}

# Build instructions:
#
# cargo build
