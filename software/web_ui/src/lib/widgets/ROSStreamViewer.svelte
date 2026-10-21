<script lang="ts" module>
	import type { WidgetGroupType, WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Image Stream';
	export const description = 'View a live image topic (sensor_msgs/Image).';
	export const group: WidgetGroupType = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state({
		groups: {
			stream: {
				topic: {
					type: 'text',
					value: '/camera/camera/color/image_raw'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import * as ROSLIB from 'roslib';

	// Raw images are large (a 640x480 rgb8 frame is ~1.2 MB as base64 JSON), so ask rosbridge
	// to throttle and keep only the latest frame rather than streaming at the camera rate.
	const THROTTLE_MS = 200;

	// sensor_msgs/Image as delivered by rosbridge (data is base64 or a number array)
	type ImageMsg = {
		width: number;
		height: number;
		encoding: string;
		step: number;
		data: string | number[];
	};

	let status = $state<string>('ROS disconnected');
	let hasFrame = $state(false);
	let canvas = $state<HTMLCanvasElement | null>(null);
	let imageData: ImageData | null = null;

	// Utility: base64 → Uint8Array
	function base64ToBytes(b64: string): Uint8Array {
		const bin = atob(b64);
		const out = new Uint8Array(bin.length);
		for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
		return out;
	}

	// bytes per pixel and channel order (r, g, b offsets; -1 alpha = opaque) per encoding
	const ENCODINGS: Record<string, { bpp: number; r: number; g: number; b: number; a: number }> = {
		rgb8: { bpp: 3, r: 0, g: 1, b: 2, a: -1 },
		bgr8: { bpp: 3, r: 2, g: 1, b: 0, a: -1 },
		rgba8: { bpp: 4, r: 0, g: 1, b: 2, a: 3 },
		bgra8: { bpp: 4, r: 2, g: 1, b: 0, a: 3 },
		mono8: { bpp: 1, r: 0, g: 0, b: 0, a: -1 }
	};

	const drawFrame = (msg: ImageMsg, topicName: string) => {
		if (!canvas) return;
		const { width, height } = msg;
		const encoding = String(msg.encoding).toLowerCase();
		const layout = ENCODINGS[encoding];
		if (!layout) {
			status = `Unsupported encoding: ${encoding}`;
			return;
		}
		if (!Number.isInteger(width) || !Number.isInteger(height) || width <= 0 || height <= 0) {
			return;
		}

		const ctx = canvas.getContext('2d');
		if (!ctx) return;

		// Only resize (which clears the canvas and reallocates) when the frame size changes.
		if (canvas.width !== width || canvas.height !== height || !imageData) {
			canvas.width = width;
			canvas.height = height;
			imageData = ctx.createImageData(width, height);
		}

		const bytes = typeof msg.data === 'string' ? base64ToBytes(msg.data) : new Uint8Array(msg.data);
		const step = msg.step || width * layout.bpp; // row stride may include padding
		if (bytes.length < step * (height - 1) + width * layout.bpp) return; // truncated frame

		const dst = imageData.data;
		for (let y = 0; y < height; y++) {
			let i = y * step;
			let j = y * width * 4;
			for (let x = 0; x < width; x++, i += layout.bpp, j += 4) {
				dst[j] = bytes[i + layout.r];
				dst[j + 1] = bytes[i + layout.g];
				dst[j + 2] = bytes[i + layout.b];
				dst[j + 3] = layout.a < 0 ? 255 : bytes[i + layout.a];
			}
		}

		// Draw straight onto the displayed canvas (no per-frame PNG encode / data URL).
		ctx.putImageData(imageData, 0, 0);
		hasFrame = true;
		status = `Streaming: ${topicName}`;
	};

	$effect(() => {
		const ros = getRosConnection();
		const topicName = settings.groups.stream.topic.value?.trim();

		if (!ros) {
			status = 'ROS disconnected';
			return;
		}
		if (!topicName) {
			status = 'No topic set';
			return;
		}

		status = `Subscribing: ${topicName}`;
		hasFrame = false;

		const sub = new ROSLIB.Topic<ImageMsg>({
			ros,
			name: topicName,
			messageType: 'sensor_msgs/msg/Image',
			throttle_rate: THROTTLE_MS,
			queue_length: 1
		});
		sub.subscribe((msg) => drawFrame(msg, topicName));

		// runs on topic/connection change and on unmount
		return () => {
			try {
				sub.unsubscribe();
			} catch {}
		};
	});
</script>

<div class="flex h-full w-full flex-col gap-2">
	<div class="text-xs opacity-70">{status}</div>

	<div class="relative min-h-0 flex-1 overflow-hidden rounded-xl border bg-black/30">
		<canvas
			bind:this={canvas}
			class="h-full w-full object-contain"
			class:invisible={!hasFrame}
			aria-label="ROS image stream"
		></canvas>
		{#if !hasFrame}
			<div
				class="absolute inset-0 flex h-full w-full items-center justify-center text-xs opacity-60"
			>
				No frames yet
			</div>
		{/if}
	</div>
</div>
