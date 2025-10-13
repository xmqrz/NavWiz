<script>
  import { onDestroy, onMount } from 'svelte';
  import * as _ from 'lodash-es';
  import { TreeView } from '@skeletonlabs/skeleton';

  import { statusChannel } from 'stores/sock/index.js';
  import StatusItem from './StatusItem.svelte';

  function merger(objValue, srcValue, key) {
    if (key === 'children' && Array.isArray(objValue) && Array.isArray(srcValue)) {
      let objMap = {};
      for (let v of objValue) {
        if (v.name) {
          objMap[v.name] = v;
        }
      }

      let res = [];
      for (let v of srcValue) {
        if (v.name) {
          res.push(_.mergeWith(objMap[v.name], v, merger));
        }
      }

      res.collapsed = objValue.collapsed;
      return res;
    }
  }

  let diagnostics = {};

  function statusCallback(data) {
    if (data.id === 'diagnostics') {
      diagnostics = _.mergeWith(diagnostics, data.diagnostics, merger);
    }
  }

  function getTimestamp(diagnostics) {
    if (diagnostics.timestamp) {
      let d = new Date(diagnostics.timestamp * 1000);
      return d.toString();
    } else {
      return '-';
    }
  }

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

  onMount(() => {
    statusChannel.subscribe(statusCallback);
  });

  onDestroy(() => {
    statusChannel.unsubscribe(statusCallback);
  });
</script>

<div class="flex h-full flex-col">
  <div class="item item-divider item-light py-2 pb-4 text-gray-700">
    Timestamp:
    <i class={getIconClass(diagnostics.status)}></i>
    <span>{getTimestamp(diagnostics)}</span>
  </div>
  {#if diagnostics.status}
    <div class="flex-grow rounded-xl bg-gray-100 p-4">
      <TreeView
        caretOpen=""
        caretClosed="-rotate-90"
        hyphenOpacity="opacity-30"
        regionSymbol="opacity-30"
        indent="pl-7"
        class="select-none"
        spacing="space-y-0"
        regionSummary="!py-2 !mb-2 bg-white shadow-lg">
        {#each diagnostics.status.children as item}
          <StatusItem {item} level={0} />
        {/each}
      </TreeView>
    </div>
  {/if}
</div>
