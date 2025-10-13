<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { goto } from '$app/navigation';
  import webhooks from '$lib/shared/services/config/webhooks';

  export let data;

  const form = {
    add: true,
    title: 'Add new webhook',
    events: [],
    error: {}
  };

  const event_types = ['"agv_update"', '"task_create"', '"task_update"'];

  if ('url' in data) {
    form.add = false;
    form.title = 'Edit webhook "' + data.url + '"';
  } else {
    data = {
      url: '',
      verify_ssl: false,
      secret_token: '',
      events: ''
    };
  }
  form.events = data.events.split(',');

  async function handleSubmit() {
    try {
      const _res = form.add ? await webhooks.add(data) : await webhooks.update(data.id, data);
      goto(webhooks.listUrl(), { replaceState: true });
    } catch (err) {
      let response = err.cause;
      if (response.status === 400) {
        // Bad Request
        form.error = await response.json();
      }
    }
  }

  const formLabelClass = 'text-lg font-semibold pr-4 text-right';
</script>

<ConfigLayout title={form.title} back={['Webhooks', webhooks.listUrl()]} validation={false}>
  <form class="w-full space-y-4 lg:w-3/4" on:submit|preventDefault={handleSubmit}>
    <fieldset class="space-y-4 rounded p-3 px-10">
      <label class="grid grid-cols-4">
        <span class="{formLabelClass} {'url' in form.error ? 'text-error-600' : ''}">
          URL
        </span>
        <input
          required
          class="input {'url' in form.error ? 'input-error' : ''} col-span-3"
          type="url"
          maxlength="200"
          bind:value={data.url} />
      </label>
      {#if 'url' in form.error}
        <div class="grid grid-cols-4">
          <strong class="col-span-3 col-start-2 text-error-600">{form.error.url}</strong>
        </div>
      {/if}

      <div class="grid grid-cols-4">
        <label class="col-span-3 col-start-2 space-x-2">
          <input
            class="checkbox {'verify_ssl' in form.error ? 'input-error' : ''}"
            type="checkbox"
            bind:checked={data.verify_ssl} />
          <span class="{formLabelClass} {'verify_ssl' in form.error ? 'text-error-600' : ''}">
            Enable SSL Verification
          </span>
        </label>
        {#if 'verify_ssl' in form.error}
          <strong class="col-span-3 col-start-2 text-error-600">
            {form.error.verify_ssl}
          </strong>
        {/if}
      </div>

      <label class="grid grid-cols-4">
        <span class="{formLabelClass} {'secret_token' in form.error ? 'text-error-600' : ''}">
          Secret Token
        </span>
        <input
          class="input {'secret_token' in form.error ? 'input-error' : ''} col-span-3"
          type="text"
          bind:value={data.secret_token} />
        <br />
        <span class="col-span-3 text-gray-500">
          Use this token to validate received payloads. It will be sent with the request in the
          X-Navwiz-Token HTTP header.
        </span>
        {#if 'secret_token' in form.error}
          <br />
          <span class="font-semibold text-error-600">{form.error.secret_token}</span>
        {/if}
      </label>

      <div class="grid grid-cols-4">
        <span class="{formLabelClass} {'events' in form.error ? 'text-error-600' : ''}">
          Events
        </span>
        <div class="col-span-3 col-start-2">
          {#each event_types as evt}
            <label class="space-x-2">
              <input
                class="checkbox {'events' in form.error ? 'input-error' : ''}"
                type="checkbox"
                value={evt}
                bind:group={form.events}
                on:change={() => {
                  data.events = form.events.join(',');
                }} />
              <span class={'events' in form.error ? 'text-error-600' : ''}>
                {evt.replaceAll('"', '')}
              </span>
            </label>
          {/each}
        </div>
        {#if 'events' in form.error}
          <strong class="col-span-3 col-start-2 text-error-600">{form.error.events}</strong>
        {/if}
      </div>
    </fieldset>
    <div class="grid grid-cols-4 p-3 px-10">
      <div class="col-span-3 col-start-2">
        <button type="submit" class="variant-filled-primary btn"> Submit </button>
      </div>
    </div>
  </form>
</ConfigLayout>
