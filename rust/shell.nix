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
in with nixpkgs; let
  rustChannel = rustChannelOfTargets
    "nightly"
    "2021-07-28"
    [ "x86_64-unknown-linux-gnu" "thumbv6m-none-eabi" ];

in stdenv.mkDerivation {
  name = "rust";
  buildInputs = 
    let
    in [ rustChannel gcc-arm-embedded ] ++
         (with nixpkgs; [ pkgconfig zlib libusb openocd ]);
  TARGET_CC = "${nixpkgs.gcc-arm-embedded}/bin/arm-none-eabi-gcc";
  TARGET = "thumbv6m-none-eabi";
  LD_LIBRARY_PATH = with nixpkgs; [ "${zlib}/lib" "${libusb}/lib" ];
  PKG_CONFIG_PATH = with nixpkgs; [ "${zlib}/lib/pkgconfig" "${libusb.dev}/lib/pkgconfig" ];
}

# Build instructions:
#
# rustup target add $TARGET
# rustup update
# cargo rustc --release --target thumbv6m-none-eabi -- -C link-arg=-Tlink.x
#
