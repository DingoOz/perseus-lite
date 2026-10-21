import { Entity, Fields } from "remult";

@Entity("layout", {
  // Unauthenticated CRUD: the UI saves layouts through this API. Acceptable only because
  // the web UI is a LAN tool on the robot network; widget `state` is treated as untrusted
  // on load (see mergePersistedSettings in state.svelte.ts).
  allowApiCrud: true,
})
export class Layout {
  @Fields.uuid()
  id!: string;

  @Fields.string()
  title?: string;

  @Fields.string()
  description?: string;

  @Fields.boolean({ defaultValue: () => false })
  isLocked?: boolean;

  @Fields.json()
  widgets: Array<{
    name: string;
    x: number;
    y: number;
    w: number;
    h: number;
    state?: string;
  }> = [];
}
