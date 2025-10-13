import {
  computePosition,
  autoUpdate,
  flip,
  shift,
  offset as offsetMiddleware
} from '@floating-ui/dom';

// floating utils
export const floatingType = {
  hover: 0,
  click: 1
};
export function floating(
  targetDom,
  floatDom,
  type = floatingType.click,
  placement = 'bottom',
  offset = 5
) {
  // init floatDom
  Object.assign(floatDom.style, {
    display: 'none',
    position: 'fixed',
    top: '0',
    left: '0',
    'z-index': '50'
  });
  let cancelFloat;
  function floatEnter() {
    if (cancelFloat) {
      cancelFloat();
      cancelFloat = undefined;
    }
    cancelFloat = autoUpdate(targetDom, floatDom, () => {
      computePosition(targetDom, floatDom, {
        placement,
        middleware: [offsetMiddleware(offset + 1), flip(), shift({ offset })]
      }).then(({ x, y }) => {
        Object.assign(floatDom.style, {
          left: `${x}px`,
          top: `${y}px`
        });
      });
    });
    floatDom.style.display = 'block';
  }
  function floatLeave() {
    floatDom.style.display = 'none';
    if (cancelFloat) {
      cancelFloat();
      cancelFloat = undefined;
    }
  }
  if (type === floatingType.hover) {
    targetDom.addEventListener('mouseenter', floatEnter);
    targetDom.addEventListener('focus', floatEnter);
    targetDom.addEventListener('mouseleave', floatLeave);
    targetDom.addEventListener('blur', floatLeave);
  } else if (type === floatingType.click) {
    targetDom.addEventListener('click', floatEnter);
    targetDom.addEventListener('blur', floatLeave);
    targetDom.addEventListener('keydown', function (e) {
      if (e.key === 'Escape') {
        floatLeave();
      }
    });
  }

  return {
    show: floatEnter,
    hide: floatLeave
  };
}
