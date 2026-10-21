<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'Joystick Teleop';
	export const description = 'Control the rover with a joystick';
	export const group = 'ROS';
	export const isRosDependent = true;

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {
			General: {
				joyStickRadius: {
					type: 'number',
					description: 'Joystick Radius in pixels',
					value: '64'
				},
				linearSpeedMultiplier: {
					type: 'number',
					description: 'Forwards and backwards speed multiplier for the joystick',
					value: '1'
				},
				angularSpeedMultiplier: {
					type: 'number',
					description: 'Left and right speed multiplier for the joystick',
					value: '0.5'
				}
			}
		}
	});
</script>

<script lang="ts">
	import { getRosConnection } from '$lib/scripts/rosBridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	import * as ROSLIB from 'roslib';
	import { onMount } from 'svelte';
	import Button from '$lib/components/ui/button/button.svelte';
	import type { TwistStampedType } from '$lib/scripts/rosTypes';

	// wrapper for widget setting
	let joystickRadius = $derived<number>(Number(settings.groups.General.joyStickRadius.value));

	// How long the joystick stays unlocked while idle before it re-locks itself.
	const LOCK_TIMEOUT_MS = 10000;
	// Publish period while the joystick is held. Must be well under twist_mux's 0.5 s timeout.
	const PUBLISH_PERIOD_MS = 75;

	let topic: ROSLIB.Topic<TwistStampedType> | null = null;
	let intervalHandle: ReturnType<typeof setInterval> | null = null;
	let lockingTimeoutHandle: ReturnType<typeof setTimeout> | null = null;
	let unlockTimeoutHandle: ReturnType<typeof setTimeout> | null = null;
	let joystickValue = { x: 0, y: 0 };
	let mousePosition = { x: 0, offsetX: 0, y: 0, offsetY: 0 };
	let unlocked = $state<boolean>(false);
	let data = $state<string>();
	let joystick = $state<{
		active: boolean;
		container: HTMLDivElement | null;
		origin: { x: number; y: number };
	}>({
		active: false,
		container: null,
		origin: { x: 0, y: 0 }
	});

	// HTML components for the joystick
	const joystickHandle = document.createElement('div');
	const joystickBase = document.createElement('div');
	joystickHandle.classList.add('bg-red-500', 'opacity-50', 'rounded-full', 'absolute', 'z-30');
	joystickBase.classList.add('bg-red-500', 'opacity-50', 'rounded-[10%]', 'absolute', 'z-20');

	// rosbridge does NOT fill in an empty header (it would publish stamp 0), so stamp the
	// message here with the browser's wall clock.
	const makeStamp = () => {
		const ms = Date.now();
		return { sec: Math.floor(ms / 1000), nanosec: (ms % 1000) * 1_000_000 };
	};

	const publishTwist = (linearX: number, angularZ: number) => {
		if (!topic) return;
		topic.publish({
			header: { stamp: makeStamp(), frame_id: 'base_link' },
			twist: {
				linear: { x: linearX, y: 0, z: 0 },
				angular: { x: 0, y: 0, z: angularZ }
			}
		});
	};

	const updateJoystick = (x: number, y: number) => {
		joystickHandle.style.left = `calc(${x}px - ${joystickRadius / 8}px)`;
		joystickHandle.style.top = `calc(${y}px - ${joystickRadius / 8}px)`;
		joystick.container?.appendChild(joystickHandle);
	};

	const hideJoystick = () => {
		joystickValue.x = 0;
		joystickValue.y = 0;
		joystick.active = false;

		const container = joystick.container;
		if (!container) return;

		const forRemove: Element[] = [];
		for (const child of container.children) {
			if (child.nodeName.toLowerCase() === 'div') forRemove.push(child);
		}

		for (const child of forRemove) {
			container.removeChild(child);
		}
	};

	// Release the joystick and, if it was driving, publish a zero twist so the rover stops
	// immediately instead of relying on the twist_mux / controller timeouts.
	const stopDriving = () => {
		const wasActive = joystick.active;
		hideJoystick();
		if (wasActive) publishTwist(0, 0);
	};

	const clearLockTimer = () => {
		if (lockingTimeoutHandle) clearTimeout(lockingTimeoutHandle);
		lockingTimeoutHandle = null;
	};

	const cancelUnlock = () => {
		if (unlockTimeoutHandle) clearTimeout(unlockTimeoutHandle);
		unlockTimeoutHandle = null;
	};

	// Lock the joystick: cancels any pending unlock, stops driving and sends a zero twist.
	// Used for window blur, tab hidden, idle timeout and unmount.
	const lock = () => {
		cancelUnlock();
		clearLockTimer();
		unlocked = false;
		stopDriving();
	};

	const scheduleLock = () => {
		clearLockTimer();
		lockingTimeoutHandle = setTimeout(() => {
			lockingTimeoutHandle = null;
			lock();
		}, LOCK_TIMEOUT_MS);
	};

	const deadbandAxis = (value: number, deadband: number) => {
		let clamped: boolean = value < deadband && value > -deadband;
		if (clamped) value = 0;
		else if (value > deadband) value -= deadband;
		else if (value < deadband) value += deadband;

		// re-normalise the axis
		if (!clamped) {
			if (value > 0) value /= 1 - deadband;
			else value /= 1 - deadband;
		}

		return Math.round(value * 1000) / 1000;
	};

	const onStart = (e: PointerEvent) => {
		if (!unlocked) return;

		// set the joystick size before displaying
		joystickBase.style.width = `${joystickRadius}px`;
		joystickBase.style.height = `${joystickRadius}px`;
		joystickHandle.style.width = `${joystickRadius / 4}px`;
		joystickHandle.style.height = `${joystickRadius / 4}px`;

		joystick.origin = { x: e.offsetX, y: e.offsetY };
		mousePosition = {
			x: e.offsetX,
			offsetX: e.clientX - e.offsetX,
			y: e.offsetY,
			offsetY: e.clientY - e.offsetY
		};

		// display the joystick
		joystick.active = true;
		joystickBase.style.left = `calc(${e.clientX}px - ${e.clientX - e.offsetX + joystickRadius / 2}px)`;
		joystickBase.style.top = `calc(${e.clientY}px - ${e.clientY - e.offsetY + joystickRadius / 2}px`;
		joystick.container?.appendChild(joystickBase);
		updateJoystick(mousePosition.x, mousePosition.y);

		// don't re-lock while the joystick is held
		clearLockTimer();

		// debug
		data =
			`<p>joystick position: ${joystick.origin.x}, ${joystick.origin.y}.</br>` +
			`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>`;
	};

	const onMove = (e: PointerEvent) => {
		if (!joystick.active) return;

		// update mouse position
		mousePosition.x = e.clientX - mousePosition.offsetX;
		mousePosition.y = e.clientY - mousePosition.offsetY;

		// calculate the distance from the joystick origin
		let dx = mousePosition.x - joystick.origin.x;
		let dy = mousePosition.y - joystick.origin.y;
		let halfRadius = joystickRadius / 2;
		if (dx > halfRadius) dx = halfRadius;
		if (dx < -halfRadius) dx = -halfRadius;
		if (dy > halfRadius) dy = halfRadius;
		if (dy < -halfRadius) dy = -halfRadius;

		// normalise the joystick value
		joystickValue.x = dx / halfRadius;
		joystickValue.y = dy / halfRadius;

		// add dead bands
		const deadband = 0.2;
		joystickValue.x = deadbandAxis(joystickValue.x, deadband);
		joystickValue.y = deadbandAxis(joystickValue.y, deadband);

		// display the joystick
		updateJoystick(joystick.origin.x + dx, joystick.origin.y + dy);

		// debug
		data =
			`<p>joystick position: ${joystick.origin.x}, ${joystick.origin.y}.</br>` +
			`mouse position: ${mousePosition.x}, ${mousePosition.y}. mouse position offset: ${mousePosition.offsetX}, ${mousePosition.offsetY}.<p/>` +
			`<p>Thumb offsets (x: ${dx}, y: ${dy})</p>` +
			`<p>Offsets with limits: (x: ${joystickValue.x}, y: ${joystickValue.y})</p>`;
	};

	// pointerup / pointerleave / pointercancel / lostpointercapture. Runs whether or not the
	// joystick is still unlocked, so releasing always sends a zero twist.
	const onStop = () => {
		if (!joystick.active) return;
		stopDriving();

		// re-lock the joystick after a period of inactivity
		if (unlocked) scheduleLock();
	};

	const onVisibilityChange = () => {
		if (document.visibilityState === 'hidden') lock();
	};

	// Pointer listeners and the publish loop are installed exactly once per mount (not on every
	// unlock) and are torn down on unmount.
	onMount(() => {
		unlocked = false;
		const container = joystick.container;
		const stopEvents = ['pointerup', 'pointerleave', 'pointercancel', 'lostpointercapture'];

		if (container) {
			container.addEventListener('pointerdown', onStart);
			container.addEventListener('pointermove', onMove);
			for (const event of stopEvents) container.addEventListener(event, onStop);
		}

		intervalHandle = setInterval(() => {
			// Only ever publish while unlocked AND held.
			if (!unlocked || !joystick.active || !topic) return;
			publishTwist(
				-joystickValue.y * Number(settings.groups.General.linearSpeedMultiplier.value),
				-joystickValue.x * Number(settings.groups.General.angularSpeedMultiplier.value)
			);
		}, PUBLISH_PERIOD_MS);

		return () => {
			if (intervalHandle) clearInterval(intervalHandle);
			intervalHandle = null;
			// stop the rover if the widget is removed mid-drive
			lock();
			if (container) {
				container.removeEventListener('pointerdown', onStart);
				container.removeEventListener('pointermove', onMove);
				for (const event of stopEvents) container.removeEventListener(event, onStop);
			}
		};
	});

	// handle ros connection/disconnection
	$effect(() => {
		const ros = getRosConnection();
		if (!ros) {
			topic = null;
			return;
		}

		const newTopic = new ROSLIB.Topic<TwistStampedType>({
			ros: ros,
			name: '/web_vel',
			messageType: 'geometry_msgs/TwistStamped'
		});
		topic = newTopic;

		return () => {
			// Connection changing or widget unmounting: stop while the old topic is still usable.
			stopDriving();
			if (topic === newTopic) topic = null;
			newTopic.unadvertise();
		};
	});

	const startUnlockTimer = () => {
		cancelUnlock();
		unlockTimeoutHandle = setTimeout(() => {
			unlockTimeoutHandle = null;
			unlocked = true;
			// lock the joystick again if it isn't used
			scheduleLock();
		}, 1000);
	};
</script>

<svelte:window onblur={lock} />
<svelte:document onvisibilitychange={onVisibilityChange} />

<div
	bind:this={joystick.container}
	class="relative h-full w-full border bg-transparent"
	style:z-index="2"
>
	<!-- <p style:z-index="-1">{data}</p> -->
	<!-- {@html data} -->
</div>
<!-- Message must be on a lower z index so it doesn't trigger a mouse down event -->
{#if !joystick.active}
	<p
		class="absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] overflow-hidden text-xl"
		style:filter={unlocked ? 'none' : 'blur(12px)'}
		style:z-index="1"
	>
		Press and hold to start moving the joystick.
	</p>
{/if}

{#if !unlocked}
	<div
		class="absolute left-[50%] top-[50%] -translate-x-[50%] -translate-y-[50%] overflow-hidden text-xl"
		style:z-index="3"
	>
		<Button
			variant="outline"
			onpointerdown={startUnlockTimer}
			onpointerup={cancelUnlock}
			onpointerleave={cancelUnlock}
			onpointercancel={cancelUnlock}>Hold to unlock</Button
		>
	</div>
{/if}
