/*
 * Copyright (c) 2020, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

var bannerHtml = `
<aside id="laser-sensor-error" class="alert hidden variant-filled-error p-3 mb-3">
  <i class="fa-solid fa-triangle-exclamation text-4xl"></i>
  <div class="alert-message" data-toc-ignore="">
    <h3 class="h3" data-toc-ignore="">Warning</h3>
    <p class="content">
    </p>
  </div>
</aside>
`;

export default function (viz, container) {
  var banner = $(bannerHtml);
  var content = banner.find('.content');
  container.append(banner);

  function hide() {
    banner.toggleClass('hidden', true);
  }

  function show(msg) {
    content.text(msg);
    banner.toggleClass('hidden', false);
  }

  viz.error = {
    hide: hide,
    show: show
  };
}
