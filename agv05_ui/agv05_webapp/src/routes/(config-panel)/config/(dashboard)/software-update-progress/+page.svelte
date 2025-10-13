<script>
  import ConfigLayout from 'components/ConfigLayout.svelte';
  import { softwareUpdateChannel, systemSock } from 'stores/sock';
  import { onMount } from 'svelte';
  // import { page } from '$app/stores';

  import resizeHandleF from '$lib/shared/resize-handle';

  // $page.state was working in another project, not sure why here it doesn't.
  const message = history.state?.message;
  const a = history.state?.logFile;

  let display;

  onMount(() => {
    resizeHandleF(display);
    softwareUpdateChannel.subscribe(softwareUpdateCallback);

    // allow time for subscribe to happen first.
    window.setTimeout(function () {
      softwareUpdateChannel.publish({
        id: 'subscribe',
        a: a
      });
    }, 500);

    // clear previous messages on reconnection
    systemSock.on('open', function () {
      display.innerHtml = '';
      softwareUpdateChannel.publish({
        id: 'subscribe',
        a: a
      });
    });

    function softwareUpdateCallback(data) {
      if (data.id === 'line') {
        const autoScroll = display.scrollTop + display.clientHeight >= display.scrollHeight;

        let line = document.createElement('div');
        line.textContent = data.line;
        display.append(line);

        if (autoScroll) {
          display.scrollTop = display.scrollHeight;
        }
      }
    }

    return () => {
      softwareUpdateChannel.unsubscribe(softwareUpdateCallback);
    };
  });
</script>

<ConfigLayout title="Updating Software..." validation={false}>
  <div class="alert variant-filled-warning mb-3">
    <div class="alert-message">{message}</div>
  </div>
  <div
    id="progress-display"
    class="h-[450px] overflow-y-scroll whitespace-pre-wrap border border-primary-500 font-mono"
    bind:this={display}>
  </div>
</ConfigLayout>
