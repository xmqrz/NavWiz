<script>
  /*
   * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
   * All rights reserved.
   *
   * Author: Farhan Mustar
   */

  import nipplejs from 'nipplejs';
  import cashDom from 'cash-dom';
  import { twMerge } from 'tailwind-merge';

  const THRESHOLD = 0.1;
  let back;

  let className = '';
  export { className as class };
  export let onChange = undefined;
  export let onEnd = undefined;
  export let data = {};
  export let screenRotate = false;
  export let resetCenter = true;

  function rotate(val) {
    if (!screenRotate) {
      return val;
    }
    return -val;
  }

  function threshold(val) {
    return Math.abs(val) > THRESHOLD ? val : 0.0;
  }

  function handleRotateChange(value) {
    if (back) {
      if (value) {
        back.addClass('screen-rotate-active');
      } else {
        back.removeClass('screen-rotate-active');
      }
    }
  }

  $: handleRotateChange(screenRotate);

  function joystick(dom) {
    let joystickManager = nipplejs.create({
      zone: dom,
      mode: 'static',
      color: '#EF473A',
      position: { top: '50%', right: '50%' },
      size: 200,
      restOpacity: 1,
      restJoystick: resetCenter,
      dynamicPage: true
    });

    cashDom(joystickManager[0].ui.front).css({
      opacity: 1
    }); // front
    back = cashDom(joystickManager[0].ui.back)
      .css({
        color: 'white',
        'font-size': '30px',
        'text-align': 'center'
      })
      .addClass('screen-rotate')
      .html('<i class="fa-solid fa-chevron-up"></i>');

    if (screenRotate) {
      back.addClass('screen-rotate-active');
    }

    joystickManager.on('end move', function (evt, d) {
      var x = 0.0;
      var y = 0.0;

      if (evt.type === 'move') {
        x = rotate(threshold(d.vector.x));
        y = rotate(threshold(d.vector.y));
      } else if (!resetCenter) {
        x = data.x;
        y = data.y;
      }

      data.x = x;
      data.y = y;
      if (onChange) {
        onChange(data);
      }
      if (evt.type === 'end' && onEnd) {
        onEnd(data);
      }
    });

    const handleDoubleClick = function () {
      if (resetCenter) {
        return;
      }

      data.x = 0.0;
      data.y = 0.0;
      joystickManager[0].setPosition(undefined, data);
      if (onChange) {
        onChange(data);
      }
      if (onEnd) {
        onEnd(data);
      }
    };

    dom.addEventListener('dblclick', handleDoubleClick);

    return {
      destroy() {
        dom.removeEventListener('dblclick', handleDoubleClick);
        if (joystickManager) {
          joystickManager.destroy();
        }
      }
    };
  }
</script>

<div class={twMerge('pointer-events-auto relative', className)} use:joystick></div>
