<script>
  import { fly } from 'svelte/transition';
  import Keyboard from 'svelte-keyboard';
  import { get } from 'svelte/store';

  import { keyboardState as ks } from './controller.js';
  import { intKeypadKeys, doubleKeypadKeys, ipKeypadKeys } from './custom-layout.js';

  function handleStringInput(event) {
    const target = get(ks);
    const start = target.selectionStart;
    const end = target.selectionEnd;
    const input = event.detail;
    if (input === 'Backspace') {
      if (target.value.length === 0 || start === 0) {
        return;
      }
      target.value = target.value.slice(0, start - 1) + target.value.slice(start);
      target.setSelectionRange(start - 1, end - 1);
    } else if (input === 'Space') {
      target.value = target.value.slice(0, start) + ' ' + target.value.slice(start);
      target.setSelectionRange(start + 1, end + 1);
    } else if (input === 'Enter') {
      target.blur();
    } else {
      target.value = target.value.slice(0, start) + input + target.value.slice(start);
      target.setSelectionRange(start + 1, end + 1);
    }
  }

  function handleNumericInput(event) {
    const target = get(ks);
    const input = event.detail;
    let currentInput = target.value.toString();
    if (input === 'Backspace') {
      if (currentInput.length === 0) {
        return;
      }
      target.value = currentInput.slice(0, -1);
    } else if (['↑', '↓'].indexOf(input) >= 0) {
      const isFloat = target.step === 'any' || target.step.includes('.');
      let step = parseFloat(target.step);
      if (isNaN(step)) {
        step = isFloat ? 0.1 : 1;
      }
      let value = parseFloat(target.value) || 0;
      if (input === '↓') {
        value = value - step;
      } else {
        value = value + step;
      }
      const max = parseFloat(target.max);
      const min = parseFloat(target.min);
      if (!isNaN(max) && value > max) {
        value = max;
      } else if (!isNaN(min) && value < min) {
        value = min;
      }
      if (isFloat && step.toString().indexOf('.') >= 0) {
        value = value.toFixed(step.toString().split('.')[1].length);
      }
      target.value = `${value}`;
    } else if (input === 'Enter') {
      target.blur();
    } else if (input === '-' && currentInput.length > 0) {
      return;
    } else if (input === '.' && currentInput.indexOf('.') >= 0) {
      return;
    } else {
      target.value += input;
    }
  }
</script>

{#if $ks}
  {#key $ks}
    <!-- svelte-ignore a11y-no-static-element-interactions -->
    <div
      on:touchstart|preventDefault|stopPropagation
      on:mousedown|preventDefault|stopPropagation
      transition:fly={{ y: 200, duration: 300 }}
      class="variant-glass-surface fixed inset-x-0 bottom-0 z-[1000] bg-surface-700/30 p-3">
      {#if $ks.getAttribute('data-keyboard') === 'ip'}
        <div class="mx-auto max-w-2xl">
          <div class="ml-[110.335px] w-full">
            <Keyboard
              on:keydown={handleStringInput}
              custom={ipKeypadKeys}
              keyClass={{
                ' ': 'key--Shift pointer-events-none !bg-transparent'
              }}
              --background="#ffffff"
              --height="80px">
            </Keyboard>
          </div>
        </div>
      {:else if $ks.getAttribute('data-ori-type') !== 'number'}
        <div class="mx-auto max-w-6xl">
          <Keyboard
            on:keydown={handleStringInput}
            custom=""
            --background="#ffffff"
            --height="80px">
          </Keyboard>
        </div>
      {:else if $ks.step === 'any' || $ks.step.includes('.')}
        <div class="mx-auto max-w-2xl">
          <Keyboard
            on:keydown={handleNumericInput}
            custom={doubleKeypadKeys}
            keyClass={{
              '↑': 'key--Shift',
              '↓': 'key--Shift'
            }}
            --background="#ffffff"
            --height="80px">
          </Keyboard>
        </div>
      {:else}
        <div class="mx-auto max-w-2xl">
          <Keyboard
            on:keydown={handleNumericInput}
            custom={intKeypadKeys}
            keyClass={{
              '↑': 'key--Shift',
              '↓': 'key--Shift'
            }}
            --background="#ffffff"
            --height="80px">
          </Keyboard>
        </div>
      {/if}
    </div>
  {/key}
{/if}
