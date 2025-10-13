<script>
  import { getContext, onMount } from 'svelte';
  import { getAPI } from '$lib/utils';

  const pathname = '/config/network/hostname';
  const handleSubmit = getContext('networkSubmit');

  let data = {
    hostname: ''
  };

  onMount(async () => {
    data = await getAPI(pathname);
  });
</script>

<div class="container">
  <form class="w-full lg:w-3/4" on:submit|preventDefault={() => handleSubmit(data)}>
    <fieldset class="rounded p-3 px-10">
      <label class="label grid grid-cols-4">
        <span class="pr-4 text-right text-lg font-semibold">Hostname</span>
        <input
          required
          class="input col-span-3"
          type="text"
          minlength="1"
          maxlength="253"
          pattern="^([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]{'{'}0,61}[a-zA-Z0-9])(\.([a-zA-Z0-9]|[a-zA-Z0-9][a-zA-Z0-9\-]{'{'}0,61}[a-zA-Z0-9]))*$"
          title="RFC-1123"
          bind:value={data.hostname} />
        <br />
        <strong class="col-span-3 text-error-600">
          Warning: A soft reboot will be performed after updating the hostname. Please make
          sure no tasks are running.
        </strong>
      </label>
    </fieldset>
    <div class="mt-7 grid grid-cols-4 px-10">
      <div class="col-span-2 col-start-2">
        <button type="submit" class="variant-filled-primary btn">Submit</button>
      </div>
    </div>
  </form>
</div>
