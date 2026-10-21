import { type Component } from "svelte";
import { Layout } from "../../shared/Layout";

// represents the entire widget
export interface WidgetType {
  name: string;
  description?: string;
  group?: WidgetGroupType;
  isRosDependent?: boolean;
  component: Component;
  settings: WidgetSettingsType;
  layoutProps?: {
    x: number;
    y: number;
    w: number;
    h: number;
  };
}

// represents the settings of the widget
export interface WidgetSettingsType {
  groups: Record<
    string,
    Record<
      string,
      {
        type: "text" | "number" | "select" | "switch" | "button" | "readonly";
        description?: string;
        value?: string;
        options?: { value: string; label: string }[]; // these are not saved as options are typically session dependent
        disabled?: boolean;
        action?: () => string | null;
      }
    >
  >;
}

// Wrap the layouts in an object with the property: value to allow the export to be reassigned
export let layouts = $state<{ active: number; value: Array<Layout> }>({
  active: 0,
  value: [],
});

export let activeWidgets = $state<{ value: Array<WidgetType> }>({ value: [] });

export let availableWidgets = $state<Array<WidgetType>>([]);

export const getWidgetByName = (name: string): WidgetType | undefined => {
  return availableWidgets.find((widget) => widget.name === name);
};

const isRecord = (value: unknown): value is Record<string, unknown> =>
  typeof value === "object" && value !== null && !Array.isArray(value);

const isOptionList = (
  value: unknown,
): value is { value: string; label: string }[] =>
  Array.isArray(value) &&
  value.every(
    (option) =>
      isRecord(option) &&
      typeof option.value === "string" &&
      typeof option.label === "string",
  );

// Overlay persisted widget settings (a JSON string from the layout database) onto the
// widget's template. The template defines the shape: only fields that exist in the
// template are restored, and only when the persisted values have the right types, so a
// corrupt, stale or hand-edited layout can't crash the widget. Button actions always
// come from the template since functions can't be persisted.
export const mergePersistedSettings = (
  template: WidgetSettingsType,
  persisted: string,
  widgetName: string,
): WidgetSettingsType["groups"] => {
  let parsed: unknown;
  try {
    parsed = JSON.parse(persisted);
  } catch (error) {
    console.warn(`Ignoring unparsable saved state for "${widgetName}"`, error);
    return template.groups;
  }

  const persistedGroups = isRecord(parsed) ? parsed.groups : undefined;
  if (!isRecord(persistedGroups)) {
    console.warn(`Ignoring malformed saved state for "${widgetName}"`);
    return template.groups;
  }

  const merged: WidgetSettingsType["groups"] = {};
  for (const [group, fields] of Object.entries(template.groups)) {
    merged[group] = {};
    const persistedFields = persistedGroups[group];
    for (const [field, templateField] of Object.entries(fields)) {
      const result = { ...templateField };
      const saved = isRecord(persistedFields)
        ? persistedFields[field]
        : undefined;
      if (isRecord(saved) && saved.type === templateField.type) {
        if (typeof saved.value === "string") result.value = saved.value;
        if (typeof saved.disabled === "boolean")
          result.disabled = saved.disabled;
        if (isOptionList(saved.options)) result.options = saved.options;
      }
      merged[group][field] = result;
    }
  }
  return merged;
};

export const getWidgetsByLayoutId = (id: string): Array<WidgetType> => {
  const layout = layouts.value.find((layout) => layout.id === id);
  if (layout === undefined) return [];
  let widgets: Array<WidgetType> = [];

  for (const widgetState of layout.widgets) {
    const widgetTemplate = getWidgetByName(widgetState.name);

    // Widget exists if there is a template
    if (widgetTemplate) {
      // set the persisted state of each widget.
      if (widgetState.state) {
        widgetTemplate.settings.groups = mergePersistedSettings(
          widgetTemplate.settings,
          widgetState.state,
          widgetState.name,
        );
      }

      widgets.push({
        name: widgetState.name,
        description: widgetTemplate.description,
        group: widgetTemplate.group,
        isRosDependent: widgetTemplate.isRosDependent,
        component: widgetTemplate.component,
        settings: widgetTemplate.settings,
        layoutProps: {
          x: widgetState.x,
          y: widgetState.y,
          w: widgetState.w,
          h: widgetState.h,
        },
      });
    }
  }

  return widgets;
};

// The widget group property is a string from the list of groups defined in this type:
export type WidgetGroupType = "ROS" | "Gstreamer" | "Misc";
