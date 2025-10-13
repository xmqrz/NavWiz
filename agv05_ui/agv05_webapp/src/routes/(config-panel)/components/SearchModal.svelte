<script>
  import { onMount } from 'svelte';
  import { Autocomplete, getModalStore, popup } from '@skeletonlabs/skeleton';
  import { getAPI, postAPI, resourceIdToUrl } from '$lib/utils';

  // Props
  // svelte-ignore unused-export-let
  export let parent;

  const modalStore = getModalStore();

  const iconDict = {
    agv: 'fa-robot',
    'task-template-global-param': 'fa-book',
    variable: 'fa-subscript',
    'map-list': 'fa-rectangle-list',
    'map-active': 'fa-rectangle-list',
    'map-param': 'fa-book',
    'map-teleport': 'fa-code-compare fa-rotate-rotate-90',
    'map-transition-trigger': 'fa-arrow-right-arrow-left'
  };

  function getIcon(id) {
    if (id in iconDict) {
      return iconDict[id];
    } else if (id.startsWith('_ttpk_')) {
      return 'fa-list-check';
    } else if (id.startsWith('_map_')) {
      return 'fa-map';
    }
    return '';
  }

  let licenseVoid = false;
  let fmsVoid = false;
  let input = '';
  let searchField;

  let options = [];
  let results = [];

  async function onSelection(event) {
    input = event.detail.plainLabel;
    let result = await postAPI('/config/search', {
      search: event.detail.value
    });
    // TODO: for now we only display name ( need to use translations to render )
    if (!result || !result.data) {
      // TODO: we identify if response okay and display no result instead of blank
      results = [];
      return;
    }
    results = result.data.map((tuple) => {
      const [desc, name, id, item] = tuple;
      return {
        desc: desc,
        name: name,
        id: id,
        item: item
      };
    });
  }

  let popupSettings = {
    event: 'focus-click',
    target: 'popupAutocomplete',
    placement: 'bottom'
  };

  onMount(async () => {
    let search;
    try {
      search = await getAPI('/config/search');
    } catch (err) {
      if (err.status === 403) {
        licenseVoid = true;
      } else if (err.status === 406) {
        fmsVoid = true;
      }
    }
    if (!search || !search.choices) {
      return;
    }
    options = search.choices.reduce((acc, [title, s]) => {
      return acc.concat(
        s.map(([value, label], index) => {
          const plainLabel = label;
          if (index === 0) {
            label = `<div class="flex flex-col gap-2 text-left">
<div class="text-xs uppercase tracking-wide text-surface-500">${title}</div>
<div>${label}</div>
</div>`;
          }
          return { label, value, keywords: title, plainLabel };
        })
      );
    }, []);

    search = $modalStore[0]?.meta?.search;
    if (search) {
      searchField.parentElement.click();
      searchField.blur();
      await onSelection({ detail: search });
    }
  });
</script>

{#if $modalStore[0]}
  <div class="card min-h-[500px] w-full max-w-[500px] p-3">
    {#if licenseVoid || fmsVoid}
      <div class="flex items-center justify-center">
        {#if licenseVoid}
          <p>Search not available. License error.</p>
        {:else}
          <p>Search not available in DFleet mode.</p>
        {/if}
      </div>
    {:else}
      <input
        class="input my-3 px-7 py-3"
        type="search"
        name="demo"
        bind:value={input}
        bind:this={searchField}
        placeholder="Search..."
        use:popup={popupSettings} />
      <div
        class="card max-h-96 w-full max-w-md overflow-y-auto p-4"
        tabindex="-1"
        data-popup="popupAutocomplete">
        <Autocomplete bind:input {options} on:selection={onSelection} />
      </div>

      <nav class="list-nav pt-5">
        <ul>
          {#each results as r}
            <li>
              <a href={resourceIdToUrl(r.id, r.item)} data-sveltekit-reload>
                <span class="flex-auto italic">{r.desc}</span>
                <span class="font-bold">
                  <i class="fa-solid badge bg-tertiary-500 {getIcon(r.id)} mr-1"></i>
                  {r.name}
                </span>
              </a>
            </li>
          {/each}
          <!-- ... -->
        </ul>
      </nav>
    {/if}
  </div>
{/if}
