/*
 * Copyright (c) 2016, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import $ from 'cash-dom';

var menuTpl = `
<ul class="context-menu bg-surface-50-900-token drop-shadow-2xl p-2 rounded border border-gray-400" role="menu">
  <li><button type="button">!!! No menu defined !!!</button></li>
  <hr/>
  <li><button type="button">Please replace the content of this context menu.</button></li>
</ul>
`;

export default function () {
  var menu = $(menuTpl);
  var docs = $(document);

  menu.on('mousedown', function (evt) {
    evt.stopPropagation();
  });

  $(document.body)
    .append(menu)
    .on('mousedown mouseup', function () {
      hideMenu();
    });

  function setContents(contents) {
    menu.empty();
    menu.append(contents);
  }

  function hideMenu() {
    menu.css({
      display: 'none'
    });
    docs.off('keyup');
  }

  function showMenu(pageX, pageY) {
    menu.css({
      display: 'block',
      left: `${pageX}px`,
      top: `${pageY}px`
    });
    docs.on('keyup', onKeyup);
  }

  function onKeyup(e) {
    // esc key
    if (e.keyCode === 27) {
      hideMenu();
    }
  }

  return {
    setContents: setContents,
    hide: hideMenu,
    show: showMenu
  };
}
