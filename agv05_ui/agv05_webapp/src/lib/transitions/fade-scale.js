import { cubicInOut } from 'svelte/easing';

export default function fadeScale(
  node,
  { delay = 0, duration = 200, easing = cubicInOut, noFade = false }
) {
  const o = +getComputedStyle(node).opacity;
  const m = parseFloat(getComputedStyle(node).height);

  return {
    delay,
    duration,
    easing,
    css: (t) => {
      const eased = easing(t);
      return `opacity: ${(noFade ? 1 : eased) * o}; height:${eased * m}px;`;
    }
  };
}
