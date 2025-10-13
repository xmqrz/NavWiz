import {
  computePosition,
  autoUpdate,
  flip,
  shift,
  offset as offsetMiddleware
} from '@floating-ui/dom';

// tooltip element

let tooltipElement;
let tooltipCleanup;

export function initTooltip(target) {
  if (!tooltipElement) {
    tooltipElement = document.createElement('div');
    tooltipElement.setAttribute('id', 'agv05-tooltip');
    tooltipElement.setAttribute('role', 'tooltip');
    document.body.appendChild(tooltipElement);
    tooltipElement.style.display = 'none';
    tooltipElement.style.zIndex = '9999';
  }
  target = target ? target : document.body;
  target.addEventListener('mouseenter', tooltipEnter, true);
  target.addEventListener('focus', tooltipEnter, true);
  target.addEventListener('mouseleave', tooltipLeave, true);
  target.addEventListener('blur', tooltipLeave, true);
}

function tooltipEnter(e) {
  const target = e.target;
  const title = target.getAttribute('tip-title');
  if (!title) {
    return;
  }
  const placement = target.getAttribute('tip-placement') || 'bottom';
  const offset = parseInt(target.getAttribute('tip-offset') || '5');
  tooltipElement.innerText = title;
  tooltipElement.style.display = 'block';

  if (tooltipCleanup) {
    tooltipCleanup();
    tooltipCleanup = undefined;
  }
  tooltipCleanup = autoUpdate(target, tooltipElement, () => {
    computePosition(target, tooltipElement, {
      placement,
      middleware: [offsetMiddleware(offset + 1), flip(), shift({ offset })]
    }).then(({ x, y }) => {
      if (!document.contains(target)) {
        tooltipLeave({ target });
        return;
      }
      Object.assign(tooltipElement.style, {
        left: `${x}px`,
        top: `${y}px`
      });
    });
  });
}

function tooltipLeave(e) {
  const target = e.target;
  const title = target.getAttribute('tip-title');
  if (!title) {
    return;
  }
  tooltipElement.style.display = 'none';
  if (tooltipCleanup) {
    tooltipCleanup();
    tooltipCleanup = undefined;
  }
}
