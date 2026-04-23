{
  system-manager,
  nixpkgs-unstable,
  ...
}@inputs:
let
  buildConfig =
    {
      system,
      hostname,
      ...
    }@systemConfig:
    let
      pkgs-unstable = nixpkgs-unstable.legacyPackages.${system};
    in
    {
      ${hostname} = system-manager.lib.makeSystemConfig {
        overlays = [
          (final: prev: { unstable = pkgs-unstable; })
          system-manager.overlays.default
        ];
        modules = [
          ./system.nix
        ];
        extraSpecialArgs = {
          inherit inputs systemConfig;
        };
      };
    };
  buildConfigs =
    configs: builtins.foldl' (acc: new: acc // new) { } (builtins.map buildConfig configs);
in
buildConfigs [
  {
    system = "aarch64-linux";
    hostname = "default";
  }
]
