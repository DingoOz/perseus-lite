{
  system-manager,
  ...
}:
system-manager.lib.makeSystemConfig {
  modules = [
    ./configuration.nix
    ./system.nix
  ];
}

