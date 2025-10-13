function ipValidator() {
  return function ip(value) {
    if (!value) return 'Please enter an IP address';

    const octets = value.split('.');
    if (octets.length !== 4) return 'Please enter a valid IPv4 address';

    for (let octet of octets) {
      if (!/^\d+$/.test(octet)) return 'Each octet must be a number';
      if (octet.length > 1 && octet.startsWith('0'))
        return 'Each octet must not have leading zeros';
      const num = parseInt(octet, 10);
      if (num < 0 || num > 255) return 'Each octet must be between 0 and 255';
    }

    return true;
  };
}

function netmaskValidator() {
  const ipValidatorFunc = ipValidator();
  return function netmask(value) {
    if (!value) return 'Please enter a netmask';
    const isValidIP = ipValidatorFunc(value);
    if (isValidIP !== true) return 'Invalid netmask';
    const octets = value.split('.');
    const binary = octets
      .map((octet) => parseInt(octet, 10).toString(2).padStart(8, '0'))
      .join('');
    const firstOne = binary.indexOf('0');
    const lastZero = binary.lastIndexOf('1');
    if (firstOne > lastZero) return true;
    return 'Invalid netmask';
  };
}

export { ipValidator, netmaskValidator };
