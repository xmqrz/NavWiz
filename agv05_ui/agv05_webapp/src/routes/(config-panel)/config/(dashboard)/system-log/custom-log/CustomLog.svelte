<script>
  import { unixFormat } from 'stores/server-clock';
  import { onMount } from 'svelte';
  import * as _ from 'lodash-es';
  import cashDom from 'cash-dom';

  import resizeHandleF from '$lib/shared/resize-handle';
  import log from '$lib/shared/services/config/log';
  import { title } from 'stores/page-title';

  import Select from 'components/Select.svelte';

  export let logChannel;
  export let downloadable = true;

  title.set('Custom Log');

  let levelFilter = ['INFO', 'WARNING', 'ERROR'];
  const levels = ['INFO', 'WARNING', 'ERROR'];
  const levelOptions = [
    ['INFO', 'INFO'],
    ['WARNING', 'WARNING'],
    ['ERROR', 'ERROR']
  ];

  let display;
  let autoScroll = true;

  onMount(() => {
    logChannel.subscribe(logCallback);

    return () => {
      logChannel.unsubscribe(logCallback);
    };
  });

  function logCallback(data) {
    if (data.id === 'line') {
      let [unixtime, level] = data.line.split(' ', 2);
      let msg = data.line.slice(unixtime.length + level.length + 2);

      let time = unixFormat(unixtime);

      if (!time) {
        return;
      }

      if (levels.indexOf(level) < 0) {
        return;
      }

      let bgClass =
        level === 'ERROR'
          ? 'bg-red-300'
          : level === 'WARNING'
            ? 'bg-amber-100'
            : 'bg-slate-200';
      let pLevel = _.padEnd(level, 7);

      let line = cashDom('<div>')
        .addClass(bgClass)
        .addClass('px-2')
        .text(`${time}    ${pLevel}    ${msg}`);
      if (levelFilter.indexOf(level) < 0) {
        line.hide();
      }
      display.append(line[0]);

      if (autoScroll) {
        display.scrollTop = display.scrollHeight;
      }
    }
  }

  function updateLevelFilter() {
    let d = cashDom(display);
    if (levelFilter.indexOf('INFO') < 0) {
      d.find('.bg-slate-200').hide();
    } else {
      d.find('.bg-slate-200').show();
    }
    if (levelFilter.indexOf('WARNING') < 0) {
      d.find('.bg-amber-100').hide();
    } else {
      d.find('.bg-amber-100').show();
    }
    if (levelFilter.indexOf('ERROR') < 0) {
      d.find('.bg-red-300').hide();
    } else {
      d.find('.bg-red-300').show();
    }
  }

  function downloadCustomLog() {
    log.get();
  }
</script>

<section id="agv05-log-custom" class="space-y-4">
  <div>
    <div
      bind:this={display}
      id="log-display"
      class="overflow-y-scroll whitespace-pre-wrap font-mono"
      style="height:40vh; border:1px solid gray;"
      use:resizeHandleF>
    </div>
  </div>
  <div>
    <label>
      <input
        class="checkbox"
        type="checkbox"
        id="input-auto-scroll"
        name="auto_scroll"
        bind:checked={autoScroll} /> Auto-scroll
    </label>
  </div>
  <div>
    <span>Message Filter</span>
    <Select
      on:change={updateLevelFilter}
      multiple={true}
      bind:value={levelFilter}
      enableFilter={false}
      buttonClass="px-7 mx-0 variant-form rounded-token max-w-[400px]"
      placement="top"
      options={levelOptions} />
  </div>
  {#if downloadable}
    <div>
      <button type="button" class="variant-filled btn" on:click={downloadCustomLog}>
        <i class="fa fa-download mr-2"></i> Download custom log
      </button>
    </div>
  {/if}
</section>

<style>
  @import '$lib/styles/viz.css';
</style>
