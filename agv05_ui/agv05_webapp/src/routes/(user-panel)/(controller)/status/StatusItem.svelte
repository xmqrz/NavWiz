<script>
  import { TreeViewItem } from '@skeletonlabs/skeleton';

  export let item;
  export let level;

  const _iconClasses = [
    'fa-solid fa-circle-check text-green-500',
    'fa-solid fa-circle-exclamation text-yellow-500',
    'fa-solid fa-circle-xmark text-red-500',
    'fa-solid fa-mug-hot text-black'
  ];
  function getIconClass(item) {
    if (item && item.level in _iconClasses) {
      return _iconClasses[item.level];
    } else {
      return _iconClasses[_iconClasses.length - 1];
    }
  }
</script>

{#if item.children.length > 0 || item.values.length > 0}
  <TreeViewItem open={level < 2} hover="hover:bg-gray-50">
    <p class="item-value">
      {item.name}:
      <i class={getIconClass(item)}></i>
      {item.message}
    </p>
    <svelte:fragment slot="children">
      {#if item.children.length > 0}
        {#each item.children as childItem}
          <svelte:self item={childItem} level={level + 1} />
        {/each}
      {:else}
        <TreeViewItem class="max-w-max" padding="p-4 pr-10" hover="" hyphenOpacity="opacity-0">
          <p class="item-value">
            {#each item.values as kv}
              <span class="block">{kv.key + ': ' + kv.value}</span>
            {/each}
          </p>
        </TreeViewItem>
      {/if}
    </svelte:fragment>
  </TreeViewItem>
{:else}
  <TreeViewItem caretClosed="" hover="hover:bg-gray-50">
    <p class="item-value">
      {item.name}:
      <i class={getIconClass(item)}></i>
      {item.message}
    </p>
  </TreeViewItem>
{/if}
