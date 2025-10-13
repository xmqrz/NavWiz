/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan Mustar
 */

import * as fflate from 'fflate';
import papa from 'papaparse';
import perf from '$lib/shared/perf';
import { parse, isValid } from 'date-fns';

function DiagnosticData(options) {
  /* DiagnosticData class */
  if (!(this instanceof DiagnosticData)) {
    return new DiagnosticData(options);
  }
  options = options || {};
  this.headers = options.headers || [];
  this.headersName = options.headersName || [];
  this.data = options.data || [];
  this.length = this.data.length;

  this.get = function (idx) {
    let d = this.data[idx];
    if (!d) {
      return [];
    }
    if (this.headers.length === 0) {
      return d;
    }
    return this.headers.map((i) => d[i]);
  };

  this.slice = function (startIdx, endIdx) {
    endIdx = endIdx || this.data.length;
    let out = [];
    for (let i = startIdx; i < endIdx; i++) {
      out.push(this.get(i));
    }
    return out;
  };

  this.forEach = function (callback) {
    for (let i = 0; i < this.data.length; i++) {
      callback(this.get(i), i);
    }
  };

  this.map = function (callback) {
    let out = [];
    for (let i = 0; i < this.data.length; i++) {
      out.push(callback(this.get(i), i));
    }
    return out;
  };

  this.exportCsv = function () {
    return (
      this.headersName.map((h) => `"${h}"`).join(',') +
      '\n' +
      this.map((e) => e.map((v) => `"${v}"`).join(',')).join('\n')
    );
  };
}

export default function () {
  var metadata = {
    headers: [],
    timestamps: []
  };
  var data = [];
  var startTime;

  function init(blob) {
    return (blob.type.startsWith('text/') ? readCsv(blob) : uncompress(blob))
      .then(parseCsv)
      .then(parseMetadata)
      .then(prepareData)
      .then((d) => {
        metadata.headers = d.headers;
        metadata.timestamps = d.timestamps;
        data = d.data;
        startTime = parse(d.timestamps[0], 'yyyy-MM-dd HH:mm:ss', new Date());
        if (!isValid(startTime)) {
          startTime = undefined;
        }
      });
  }

  function uncompress(data) {
    return data.arrayBuffer().then((d) => {
      perf.begin('fflate');
      let rawData = fflate.decompressSync(new Uint8Array(d));
      perf.end('fflate');

      // TODO: allow user select which range of data to read.
      if (rawData.length > 500000000) {
        console.error('File exceed browser limit. Reading limited data.');
        window.alert('File exceed browser limit. Reading limited data.');
        rawData = rawData.slice(0, 500000000);
      }
      return new TextDecoder().decode(rawData);
    });
  }

  function readCsv(data) {
    return new Promise((resolve, _reject) => {
      perf.begin('filereader');
      let reader = new FileReader();
      reader.onload = function (_e) {
        perf.end('filereader');
        resolve(reader.result);
      };

      // TODO: allow user select which range of data to read.
      if (data.size > 500000000) {
        console.error('File exceed browser limit. Reading limited data.');
        window.alert('File exceed browser limit. Reading limited data.');
        data = data.slice(0, 500000000);
      }
      reader.readAsText(data);
    });
  }

  function parseCsv(data) {
    return new Promise((resolve, _reject) => {
      perf.begin('papa');
      papa.parse(data, {
        worker: false,
        skipEmptyLines: true,
        complete: function (results) {
          perf.end('papa');
          resolve(results.data);
        }
      });
    });
  }

  function parseMetadata(data) {
    console.log('processing ' + data.length + ' lines');
    return new Promise((resolve, _reject) => {
      let headers = [];
      let curHeaders;

      let timestamps = [];
      let prevTime;

      for (let l of data) {
        if (l.length === 0) {
          continue;
        }

        if (l[0] === 'Timestamp') {
          // TODO: reconsider: is longest header cover all node.
          curHeaders = l;
          if (l.length > headers.length) {
            headers = l;
          }
        } else if (curHeaders) {
          if (curHeaders.length !== l.length) {
            console.error('error data length does not match with header');
            continue;
          }
          let timestamp = l[0];
          if (!timestamp) {
            console.error('error timestamp data does not exist');
            continue;
          }

          let time;
          try {
            // Handle single day log only so only check time to save time ;).
            time = timestamp.split(' ')[1].split(':');
            time[0] = parseInt(time[0]);
            time[1] = parseInt(time[1]);
            time[2] = parseInt(time[2]);

            if (prevTime) {
              let nTime = [prevTime[0], prevTime[1], prevTime[2] + 1];
              if (nTime[2] >= 60) {
                nTime[2] = 0;
                nTime[1] = nTime[1] + 1;
              }
              if (nTime[1] >= 60) {
                nTime[1] = 0;
                nTime[0] = nTime[0] + 1;
              }
              if (nTime[0] !== time[0] || nTime[1] !== time[1] || nTime[2] !== time[2]) {
                fillEmptyData(timestamps, prevTime, time);
              }
            }
          } catch (error) {
            console.log('error while processing the empty time gap');
            throw error;
          }

          if (!time) {
            return;
          }

          timestamps.push(timestamp);
          prevTime = time;
        } else {
          console.error('error data without header');
        }
      }
      resolve({
        data: data,
        headers: headers,
        timestamps: timestamps
      });
    });
  }

  function prepareData(d) {
    return new Promise((resolve, _reject) => {
      let curMapping;
      let curHeaders;
      let row = 0;
      let data = [];
      for (let l of d.data) {
        if (l[0] === 'Timestamp') {
          curHeaders = l;
          curMapping = null;
        } else {
          if (!curHeaders || curHeaders.length !== l.length) {
            continue;
          }

          while (!d.timestamps[row]) {
            data.push(Array(d.headers.length).fill(null));
            row++;
          }

          let r = Array(d.headers.length).fill(null);

          // generate mappings
          if (curMapping === null) {
            curMapping = [];
            let i = 0;
            for (let j in curHeaders) {
              while (d.headers[i] !== curHeaders[j]) {
                i++;
                if (i >= d.headers.length) {
                  break;
                }
              }
              if (i >= d.headers.length) {
                break;
              }
              curMapping[i] = j;
            }
          }

          // obtain data
          if (curMapping.length > 0) {
            for (const i in curMapping) {
              let j = curMapping[i];
              if (j >= 0) {
                r[i] = l[j];
              }
            }
          }
          data.push(r);
          row++;
        }
      }

      console.log(data.length + ' lines of data parsed');
      resolve({
        data: data,
        timestamps: d.timestamps,
        headers: d.headers
      });
    });
  }

  function fillEmptyData(timestamps, pTime, nTime) {
    let pHour = pTime[0];
    let pMin = pTime[1];
    let pSec = pTime[2];

    let nHour = parseInt(nTime[0]);
    let nMin = parseInt(nTime[1]);
    let nSec = nTime[2];

    let row = nSec - pSec + (nMin - pMin) * 60 + (nHour - pHour) * 3600;

    if (row <= 1) {
      console.error('Error: try to fill empty row but empty row is: ' + row);
      return;
    }
    row--;

    while (row > 0) {
      timestamps.push('');
      row--;
    }
  }

  function getData(headersIdx) {
    return new DiagnosticData({
      data: data,
      headers: headersIdx,
      headersName:
        headersIdx.length === 0 ? metadata.headers : headersIdx.map((i) => metadata.headers[i])
    });
  }

  return {
    timestamps: () => metadata.timestamps,
    headers: () => metadata.headers,
    startTime: () => startTime,

    init: init,
    getData: getData
  };
}
