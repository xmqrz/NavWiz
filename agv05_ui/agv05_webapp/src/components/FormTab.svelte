<!-- Handle hidden invalid input when using tab -->
<script>
  import { TabGroup, Tab } from '@skeletonlabs/skeleton';
  import * as _ from 'lodash-es';
  import cashDom from 'cash-dom';

  // expect list of name and data. [[name, data]]
  export let items;
  export let classAttr = '';
  let activeTab = 0;

  let onInvalid = _.debounce(
    function (e, i) {
      if (activeTab === i) {
        return;
      }
      activeTab = i;
      setTimeout(() => {
        e.target.reportValidity();
      }, 100);
    },
    300,
    { leading: true, trailing: false }
  );

  function onPanelMount(elm, i) {
    let panel = cashDom(elm);
    let inputs = panel.find('input');
    let callback = (e) => onInvalid(e, i);
    inputs.on('invalid', callback);
    return {
      destroy() {
        inputs.off('invalid', callback);
      }
    };
  }
</script>

<TabGroup class={classAttr}>
  <slot name="tab">
    {#each items as [name, _], i}
      <Tab bind:group={activeTab} {name} value={i}>
        <span>{name}</span>
      </Tab>
    {/each}
  </slot>
  <svelte:fragment slot="panel">
    {#each items as [name, data], i}
      <div class={activeTab === i ? '' : 'hidden'} use:onPanelMount={i}>
        <slot name="panel" panel={name} {data} {i} />
      </div>
    {/each}
  </svelte:fragment>
</TabGroup>
