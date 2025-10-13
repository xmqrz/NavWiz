/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

var vizHtml = `
<div class="diagnostic-table-view diagnostic-fill diagnostic-fullscreen" style="display:flex;flex-direction:column">
</div>
`;

import $ from 'cash-dom';

import toolbarF from './toolbar';
import tableF from './table';

export default function (outer) {
  let viz = $(vizHtml);

  viz.toolbar = toolbarF(viz);
  viz.table = tableF(viz);

  viz.loaded = function () {
    outer.append(viz);
    viz.table.loaded();
    viz.toolbar.loaded();
  };

  viz.close = function () {
    $('body').css('overflow-y', 'visible');
    $(window).off('resize', viz.resized);

    viz.table.close();

    viz.remove();
  };

  viz.resized = function () {
    viz.table.resized();
  };

  viz.on('models.loaded', viz.loaded);

  $(window).on('resize', viz.resized);

  $('body').css('overflow-y', 'hidden');
  return viz;
}
