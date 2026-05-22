final: prev: {
  crc = final.callPackage ./crc { };
  fd-wrapper = final.callPackage ./fd-wrapper { };
  ptr-wrapper = final.callPackage ./ptr-wrapper { };
  simple-networking = final.callPackage ./simple-networking { };
  type-demangle = final.callPackage ./type-demangle { };
}
