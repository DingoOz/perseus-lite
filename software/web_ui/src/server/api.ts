import { remultSveltekit } from "remult/remult-sveltekit";
import { Layout } from "../shared/Layout";

export const api = remultSveltekit({
  entities: [Layout],
  // The remult admin UI exposes raw DB editing with no auth; keep it off.
  admin: false,
});
