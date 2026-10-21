# Rover UI

This repo contains the UI for controlling/viewing outputs from sensors on the rover.

### Running the UI

The Rover UI is a SvelteKit app (built with `@sveltejs/adapter-node`). It uses Yarn 4
(`packageManager` in `package.json`; enable it with `corepack enable`, or run any Yarn 4.x).
Run these from this directory (`software/web_ui`):

```shell
yarn install   # install dependencies
yarn dev       # development server (Vite, --host: reachable from any device on the network)
```

For the robot / production:

```shell
yarn build     # build the app into ./build
yarn start     # serve ./build plus the socket.io camera/resource endpoints on port 3000
```

The UI talks to ROS through `rosbridge_websocket` on port 9090 of the selected ROS host
(the hardware bringup and the simulation both start it). Teleop publishes stamped twists on
`/web_vel`.

Cameras are served by the separate GStreamer helper in `pi-server/` (`yarn camera`, configure
`pi-server/config.json`). Recordings requested from the UI are restricted to plain file names
(`[A-Za-z0-9_.-]`) and are always written to `~/perseus-recordings/` on the camera host.

**Note:** The hydration errors can be ignored

### Developing

To begin developing a widget run the command: `./create-widget.sh <file-name>`. The `file-name` argument is just the name of the file that contains the widget and **NOT** the widget name. This should generate a new file `/src/lib/widgets<file-name>.svelte` with this template contents:

```svelte
<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'New Widget';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	// Widget logic goes here
</script>

<p>New component</p>
```

- The first script tag with the `module` property is a server only module. This is used here as it is only run once when the component is first loaded and it also allows for exports that are used to expose some properties. You likely will not need to write your own code here.
  - **name -** This string is the unique name of the widget that will be displayed at the top of the widget and is used to ensure duplicates of widgets are not loaded.
  - **settings -** This is the object that structures the settings/state panel of each widget. Here is an example settings object and how to access the value of each setting:

    ```ts
    export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
      groups: {
        general: {
          textSetting: {
            type: "text", // options are: text, number, switch, select, button
            value: "Some text", // This is a default value
          },
          hereIsAButton: {
            type: "button",
            action: () => {
              // action is executed on button press
              console.log("Button clicked");
              return "Success"; // string return value is printed in a toast
            },
          },
        },
        Advanced: {
          booleanSwitch: {
            type: "switch",
            description: "This is a switch", // fields with descriptions get a question mark next to their label
          },
          selectOptions: {
            type: "select",
            options: [
              { value: "1", label: "Option 1" },
              { value: "2", label: "Option 2" },
              { value: "3", label: "Option 3" },
            ],
          },
        },
      },
    });

    console.log(settings.groups.Advanced.booleanSwitch.value); // logs the state of the switch
    ```

- The second script tag is where your code should go as it is run client side when the component is rendered. The settings can be accessed here without an import and to listen for changes wrap the value you want to check in either a `$derived` or `$effect` rune. Furthermore, use `toast('message')` to send a toast from your widget.
- The remainder of the file is where mark down goes. Tailwind classes can be used on widgets for styling or a `<style></style>` block.

## TODO:

### Widgets

- Camera
- Battery
