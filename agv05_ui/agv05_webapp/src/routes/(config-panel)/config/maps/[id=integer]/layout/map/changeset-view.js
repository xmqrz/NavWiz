/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan
 */

import $ from 'cash-dom';

var changesetsOuterHtml = `
<div class="agv05-map-changesets card">
  <div class="card-header">Revision History</div>
  <section class="p-1">
    <ul class="changeset-disp space-y-1">
    </ul>
  </section>
</div>
`;

var changesetHtml = `
<button type="button" class="btn rounded w-full flex flex-col variant-filled">
  <h4 class="text-lg font-semibold"></h4>
  <p></p>
</button>
`;

var activeClass = 'variant-filled-secondary';

export default function (viz) {
  var changesetsView = $(changesetsOuterHtml);
  var changesetDisp = changesetsView.find('.changeset-disp');
  changesetsView.hide();
  var $viz = $(viz.node());
  changesetsView.insertAfter($viz);

  function show() {
    changesetsView.show();
  }

  function hide() {
    changesetsView.hide();
  }

  function load() {
    changesetDisp.empty();
    if (!viz.models) {
      return;
    }
    var changesetList = viz.models.rawChangesets();
    let firstActive = false;
    for (let changeset of changesetList) {
      var innerHtml = $(changesetHtml);
      innerHtml.find('h4').text(changeset.author || '-');

      var date = '-';
      if (changeset.created) {
        date = changeset.created.toLocaleString();
      }
      innerHtml.find('p').text(date);
      if (!firstActive) {
        firstActive = true;
        innerHtml.addClass(activeClass);
      }
      let li = $('<li>');
      li.append(innerHtml);
      changesetDisp.append(li);
      innerHtml.on(
        'click',
        {
          changeset: changeset
        },
        changesetSelected
      );
    }
  }

  function changesetSelected(event) {
    event.preventDefault();
    var changeset = event.data.changeset;
    changesetDisp.find('button').removeClass(activeClass);
    $(event.currentTarget).addClass(activeClass);
    viz.editor.loadChangeset(changeset);
    viz.scene.activeObject.set(null);
    return false;
  }

  $viz.on('models.changesetsUpdated', load);

  function showFullscreen() {
    changesetsView.addClass('agv05-fullscreen');
  }

  function hideFullscreen() {
    changesetsView.removeClass('agv05-fullscreen');
  }

  return {
    show: show,
    hide: hide,
    showFullscreen: showFullscreen,
    hideFullscreen: hideFullscreen
  };
}
