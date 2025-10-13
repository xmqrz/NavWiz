import { writable, readonly } from 'svelte/store';

const _keyboardState = writable(null);
export const keyboardState = readonly(_keyboardState);

function numericKeydown(e) {
  // hack to ensure numeric input
  const value = e.target.value;
  if (e.key === 'Backspace') {
    if (value.length === 0) {
      // do nothing
    } else if (value.length === 2 && (value.startsWith('-') || value.startsWith('.'))) {
      e.target.value = e.target.value.slice(0, -1);
    } else if (value.length === 3 && value.startsWith('-.')) {
      e.target.value = e.target.value.slice(0, -1);
    } else {
      const v = value.slice(0, -1);
      if (!isNaN(v)) {
        e.target.value = v;
      }
    }
  } else if (e.key === '-') {
    if (value === '') {
      e.target.value = '-';
    }
  } else if (
    !isNaN(e.key) ||
    (e.key === '.' && (e.target.step === 'any' || e.target.step.includes('.')))
  ) {
    const v = value + e.key;
    if (!isNaN(v) || v === '.' || v === '-.') {
      e.target.value = v;
    }
  }
  e.preventDefault();
}

function forceCursorAtEnd(e) {
  // Put the cursor at the end of the input value
  const length = e.target.value.length;
  e.target.setSelectionRange(length, length);
}

function focusin(e) {
  if (e.target.tagName !== 'INPUT') {
    return;
  }

  if (!e.target.classList.contains('keyboard')) {
    return;
  }

  const t = e.target.getAttribute('type');
  if (t === 'number') {
    // HACK: to allow -ve symbol at the beginning.
    e.target.addEventListener('keydown', numericKeydown);
    e.target.addEventListener('click', forceCursorAtEnd);
    e.target.setAttribute('data-ori-type', t);
    e.target.setAttribute('type', 'text');
    forceCursorAtEnd(e);
  }

  _keyboardState.set(e.target);
}

function focusout(e) {
  if (e.target.tagName !== 'INPUT') {
    return;
  }
  if (!e.target.classList.contains('keyboard')) {
    return;
  }

  const t = e.target.getAttribute('data-ori-type');
  if (t) {
    e.target.removeEventListener('keydown', numericKeydown);
    e.target.removeEventListener('click', forceCursorAtEnd);
    e.target.setAttribute('type', t);
  }

  // TODO: doing here not reactive, do we need reactivity?
  // notify svelte bind variable.
  e.target.dispatchEvent(new Event('input'));
  e.target.dispatchEvent(new Event('change'));

  _keyboardState.set(null);
}

export function initKeyboard(target) {
  target = target ? target : document.body;
  target.addEventListener('focusin', focusin);
  target.addEventListener('focusout', focusout);
}

_keyboardState.subscribe((input) => {
  setTimeout(() => {
    if (!document.contains(input)) {
      return;
    }
    input.scrollIntoView({
      block: 'center',
      inline: 'nearest',
      behavior: 'smooth'
    });
  }, 300);
});
