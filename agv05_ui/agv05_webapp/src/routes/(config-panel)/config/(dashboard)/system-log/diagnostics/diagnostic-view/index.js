/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import $ from 'cash-dom';

import progressBarF from './progressbar';
import loaderF from './loader';

export default function (outer) {
  var editor = $(outer);
  let running = false;

  var progressbar = progressBarF(editor);
  var loader = loaderF(editor);

  function load(url, method) {
    // To prevent multiple clicks of diagnostic view button
    if (running) {
      console.error('There is one diagnostic view currently running.');
      return;
    }

    running = true;

    progressbar
      .show()
      .then(function () {
        return loader.load(url, method, progressbar.updateProgress);
      })
      .then(function () {
        running = false;
        return progressbar.hide();
      })
      .catch(function (e) {
        running = false;
        console.error('Diagnostic error while loading data: ' + e);
        progressbar.updateProgress('Error Loading Data...');
        setTimeout(progressbar.hide, 3000);
      });
  }
  editor.find('.diagnostic-start-view').each(function () {
    let btn = $(this);
    btn.on('click', function () {
      load(btn.data('url'), btn.data('method') || 'GET');
    });
  });

  return {};
}
