<script>
  import * as _ from 'lodash-es';
  import { createEventDispatcher } from 'svelte';

  import Menu from 'components/Menu.svelte';

  const dispatch = createEventDispatcher();

  export let filterOptions = {};

  function onClick(name, val) {
    dispatch('filterClick', { name, val });
  }
</script>

<Menu btnClass="variant-filled" placement="bottom-start">
  <div slot="text" class="space-x-3">
    <span>Filter</span>
    <i class="fa-solid fa-caret-down"></i>
  </div>
  <nav class="list-nav">
    <ul>
      {#each Object.entries(filterOptions) as [name, l]}
        <li>
          <Menu btnClass="w-full" placement="right-start">
            <div slot="text" class="space-x-3">
              <span class="flex-auto">{_.startCase(name)}</span>
              <span class="badge">
                <i class="fa-solid fa-chevron-right"></i>
              </span>
            </div>
            <nav class="list-nav">
              <ul>
                {#each l as [val, title]}
                  <li>
                    <button
                      type="button"
                      class="menu-close btn w-full !px-8"
                      on:click={() => onClick(name, val)}>
                      <span>{title}</span>
                    </button>
                  </li>
                {/each}
              </ul>
            </nav>
          </Menu>
        </li>
      {/each}
    </ul>
  </nav>
</Menu>
