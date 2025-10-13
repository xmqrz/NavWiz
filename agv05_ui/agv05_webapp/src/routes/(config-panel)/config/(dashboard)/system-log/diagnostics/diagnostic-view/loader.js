/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import viewerF from './viewer';
import { getCSRFToken } from 'stores/auth.js';

export default function (outer) {
  function load(url, method, updateProgress) {
    updateProgress('Downloading...');
    let filename = url.split('/').pop();
    let options = {
      cache: 'no-store'
    };

    if (method === 'POST') {
      options = Object.assign(options, {
        method: 'POST',
        headers: {
          'X-CSRFToken': getCSRFToken()
        }
      });
    }
    return fetch(url, options)
      .then(function (res) {
        let contentDisposition = res.headers.get('content-disposition') || '';
        let match = contentDisposition.match(/filename="(.*?)"/);
        if (match && match[1]) {
          filename = match[1];
        }
        return res.blob();
      })
      .then((d) => initiateViewer(d, filename, updateProgress));
  }

  function initiateViewer(blob, filename, updateProgress) {
    updateProgress('Reading...');
    let viewer = viewerF(outer);
    return viewer.load({
      blob: blob,
      filename: filename
    });
  }

  return {
    load: load
  };
}
