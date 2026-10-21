<script lang="ts">
	// import { connect, isConnected, ros } from '$lib/scripts/ros.svelte';
	import { cn } from '$lib/utils';
	import * as Dialog from '$lib/components/ui/dialog/index';
	import * as DropdownMenu from '$lib/components/ui/dropdown-menu/index';
	import Button from './ui/button/button.svelte';
	import Label from './ui/label/label.svelte';
	import Input from './ui/input/input.svelte';
	import { OpenInNewWindow } from 'svelte-radix';
	import { localStore } from '$lib/scripts/localStore.svelte';
	import { connectRos, disconnectRos, getRosConnection } from '$lib/scripts/rosBridge.svelte';
	import ConnectionMenuMobile from './connectionMenuMobile.svelte';
	import { toast } from 'svelte-sonner';

	let props: { isMobile: boolean } = $props();

	let domainId = $state<number>(51);
	let customAddress = $state<string>('');
	let openAddressMenu = $state(false);
	let disableServerIP = $state(false);

	let address = localStore('rosAddress', 'localhost');
	// never keep an empty / non-string address (older versions could persist '')
	if (typeof address.value !== 'string' || address.value.trim() === '') {
		address.value = 'localhost';
	}

	const changeAddress = async (value?: string) => {
		let newAddress: string;
		if (value === 'server') {
			newAddress = window.location.hostname; // get ip from the server
		} else {
			newAddress = (value || customAddress).trim(); // use the custom value if there is no param
		}
		if (newAddress === '') {
			toast.error('ROS host cannot be empty');
			return;
		}

		address.value = newAddress;
		disableServerIP = value === 'server';
		disconnectRos();
		connectRos(address.value);
		openAddressMenu = false;
	};

	const submitCustomAddress = (e: SubmitEvent) => {
		e.preventDefault(); // don't let the browser submit (reload) the page
		changeAddress();
	};

	const toggleConnection = () => {
		if (getRosConnection()) {
			disconnectRos();
		} else {
			connectRos(address.value);
		}
	};
</script>

{#if props.isMobile}
	<DropdownMenu.Root>
		<DropdownMenu.Trigger class="ml-auto mr-2">
			<div
				class={cn('my-auto h-[20px] w-[20px] rounded-[50%] bg-red-600', {
					'bg-green-600': getRosConnection()
				})}
			></div>
		</DropdownMenu.Trigger>
		<DropdownMenu.Content align="end">
			<DropdownMenu.Label>ROS Connection</DropdownMenu.Label>
			<DropdownMenu.Group>
				<DropdownMenu.Item onclick={toggleConnection}>
					<p>{getRosConnection() ? 'Disconnect' : 'Connect'}</p>
				</DropdownMenu.Item>
			</DropdownMenu.Group>
			<DropdownMenu.Separator />
			<DropdownMenu.Label>ROS Host: {address.value}</DropdownMenu.Label>
			<DropdownMenu.Group>
				<DropdownMenu.Item
					disabled={address.value === 'localhost'}
					onclick={() => changeAddress('localhost')}
				>
					localhost
				</DropdownMenu.Item>
				<DropdownMenu.Item onclick={() => changeAddress('server')} disabled={disableServerIP}
					>Same as host</DropdownMenu.Item
				>
				<DropdownMenu.Item onclick={() => (openAddressMenu = true)}>
					Custom IP <OpenInNewWindow class="ml-auto h-4 w-4" />
				</DropdownMenu.Item>
			</DropdownMenu.Group>
			<DropdownMenu.Separator />
			<DropdownMenu.Label>ROS Domain: {domainId}</DropdownMenu.Label>
			<DropdownMenu.Group>
				<DropdownMenu.Item>
					Domain ID: {domainId}{domainId === 42 ? ` (prod)` : ''}{domainId === 51 ? ` (dev)` : ''}
				</DropdownMenu.Item>
			</DropdownMenu.Group>
		</DropdownMenu.Content>
	</DropdownMenu.Root>

	<Dialog.Root bind:open={openAddressMenu}>
		<Dialog.Content>
			<Label for="customAddress">Custom ROS host:</Label>
			<form class="flex space-x-2" onsubmit={submitCustomAddress}>
				<Input class="m-auto mb-0" id="customAddress" type="text" bind:value={customAddress} />
				<Button type="submit" variant="outline">Set</Button>
			</form>
		</Dialog.Content>
	</Dialog.Root>
{:else}
	<div class="mx-auto flex">
		<ConnectionMenuMobile
			{toggleConnection}
			{changeAddress}
			{address}
			{disableServerIP}
			bind:customAddress
			{domainId}
		/>
	</div>
{/if}
