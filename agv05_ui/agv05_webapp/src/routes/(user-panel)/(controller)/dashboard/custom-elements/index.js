import BottomBar from './BottomBar.svelte';
import CameraStream from './CameraStream.svelte';

export default function registerCustomElement(customElementAPI) {
  register(BottomBar, 'bottom-bar', {
    customElementAPI
  });

  register(CameraStream, 'camera-stream', function () {
    let topic = this.getAttribute('topic');
    return {
      topic
    };
  });
}

function register(SvelteClass, elementTag, props) {
  if (customElements.get(elementTag)) {
    return;
  }
  class RegisterCustomElement extends HTMLElement {
    connectedCallback() {
      let p = props;
      if (typeof props === 'function') {
        p = props.call(this);
      }
      this.e = new SvelteClass({
        target: this,
        props: p
      });
    }
    disconnectedCallback() {
      this.e.$destroy();
    }
  }
  customElements.define(elementTag, RegisterCustomElement);
}
