/*
 * Copyright (c) 2018, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Farhan
 */

export default class Changeset {
  constructor(options) {
    options = options || {};
    this.id = options.id;
    this.author = options.author || '';
    this.created = new Date(options.created || '');
  }
}

Changeset.__attrs__ = ['id', 'author', 'created'];
