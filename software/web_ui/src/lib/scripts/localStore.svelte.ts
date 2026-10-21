import { browser } from "$app/environment";

export class LocalStore<T> {
  value = $state<T>() as T;
  key = "";

  constructor(key: string, value: T) {
    this.key = key;
    this.value = value;

    if (browser) {
      const item = localStorage.getItem(key);
      if (item) {
        try {
          this.value = this.deserialize(item);
        } catch {
          // corrupt entry: keep the default (the effect below overwrites it)
          console.warn(`Ignoring unparsable localStorage entry "${key}"`);
        }
      }
    }

    $effect(() => {
      localStorage.setItem(this.key, this.serialize(this.value));
    });
  }

  serialize(value: T): string {
    return JSON.stringify(value);
  }

  deserialize(item: string): T {
    return JSON.parse(item);
  }
}

export function localStore<T>(key: string, value: T) {
  return new LocalStore(key, value);
}
