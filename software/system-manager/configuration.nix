{ ... }:
{
  nix = {
    gc = {
      automatic = true;
      dates = "weekly";
    };
    settings = {
      experimental-features = "nix-command flakes";
      auto-optimise-store = true;
      warn-dirty = false;

      trusted-substituters = [
        "https://roar-qutrc.cachix.org"
        "https://ros.cachix.org"
      ];
      extra-trusted-public-keys = [
        "roar-qutrc.cachix.org-1:ZKgHZSSHH2hOAN7+83gv1gkraXze5LSEzdocPAEBNnA="
        "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo="
      ];
    };
  };
}
