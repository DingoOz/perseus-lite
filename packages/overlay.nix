# packages/overlay.nix
final: prev: {
  groot2 = final.callPackage ./groot2 { };
  open3d = final.callPackage ./open3d { };
}
