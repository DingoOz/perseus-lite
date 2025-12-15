{
  system-manager,
  nixpkgs,
}@inputs:
system-manager.lib.makeSystemConfig {
  modules = [
    ./configuration.nix
    ./system.nix
  ]
}