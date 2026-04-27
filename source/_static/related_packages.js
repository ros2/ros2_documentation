/**
 * Populate ``.js-related-packages`` widgets from the rosdistro cache YAML.
 *
 * Depends on global ``pako`` (gzip) and ``yaml`` / ``jsyaml`` (js-yaml), loaded
 * earlier via html_js_files in conf.py.
 */
(function () {
  'use strict';

  /** @type {Record<string, Promise<Record<string, string>>>} */
  var cacheByDistro = {};

  /**
   * Resolve the js-yaml API regardless of how the bundle exposes it.
   *
   * @returns {{ load: function(string): unknown }}
   */
  function yamlApi() {
    var g = typeof window !== 'undefined' ? window : globalThis;
    /* js-yaml UMD sets ``globalThis.jsyaml`` (see dist/js-yaml.min.js). */
    if (g.jsyaml && typeof g.jsyaml.load === 'function') {
      return g.jsyaml;
    }
    if (g.yaml && typeof g.yaml.load === 'function') {
      return g.yaml;
    }
    throw new Error('js-yaml is not loaded');
  }

  /**
   * Directory containing ``related_packages.js`` (ends with slash or empty).
   *
   * @returns {string}
   */
  function scriptBaseUrl() {
    var nodes = document.getElementsByTagName('script');
    var i;
    var src;
    for (i = nodes.length - 1; i >= 0; i--) {
      src = nodes[i].src;
      if (src && src.indexOf('related_packages.js') !== -1) {
        return src.replace(/related_packages\.js([?#].*)?$/i, '');
      }
    }
    return '';
  }

  /**
   * @param {string} distro
   * @returns {string|null}
   */
  function bundledCacheUrl(distro) {
    var base = scriptBaseUrl();
    if (!base) {
      return null;
    }
    return base + 'rosdistro_cache/' + distro + '-cache.yaml.gz';
  }

  /**
   * @param {string} distro
   * @returns {Promise<Record<string, string>>}
   */
  function loadXmls(distro) {
    if (cacheByDistro[distro]) {
      return cacheByDistro[distro];
    }
    cacheByDistro[distro] = fetchAndParse(distro);
    return cacheByDistro[distro];
  }

  /**
   * @param {string} distro
   * @returns {Promise<Record<string, string>>}
   */
  function fetchAndParse(distro) {
    var remote =
      'https://repo.ros2.org/rosdistro_cache/' + encodeURIComponent(distro) + '-cache.yaml.gz';
    var urls = [];
    var bundled = bundledCacheUrl(distro);
    if (bundled) {
      urls.push(bundled);
    }
    urls.push(remote);

    return tryUrls(urls);
  }

  /**
   * @param {string[]} urls
   * @returns {Promise<Record<string, string>>}
   */
  function tryUrls(urls) {
    var i = 0;

    function next(lastErr) {
      if (i >= urls.length) {
        return Promise.reject(lastErr || new Error('failed to load rosdistro cache'));
      }
      var url = urls[i];
      i += 1;
      return fetch(url, { cache: 'no-cache' })
        .then(function (res) {
          if (!res.ok) {
            throw new Error('HTTP ' + res.status + ' for ' + url);
          }
          return res.arrayBuffer();
        })
        .then(function (buf) {
          var g = typeof window !== 'undefined' ? window : globalThis;
          var inflated = g.pako.inflate(new Uint8Array(buf), { to: 'string' });
          var data = yamlApi().load(inflated);
          var xmls = data && data.release_package_xmls;
          if (!xmls || typeof xmls !== 'object') {
            throw new Error('release_package_xmls missing in rosdistro cache');
          }
          return /** @type {Record<string, string>} */ (xmls);
        })
        .catch(function (err) {
          return next(err);
        });
    }

    return next(null);
  }

  /**
   * @param {string} xmlStr
   * @returns {string[]}
   */
  function extractBuildTypes(xmlStr) {
    var out = [];
    var re = /<build_type[^>]*>([^<]+)<\/build_type>/gi;
    var m;
    while ((m = re.exec(xmlStr)) !== null) {
      out.push(m[1].trim());
    }
    return out;
  }

  /**
   * @param {string} xmlStr
   * @param {string} want
   * @returns {boolean}
   */
  function matchesBuildType(xmlStr, want) {
    var types = extractBuildTypes(xmlStr);
    var k;
    for (k = 0; k < types.length; k += 1) {
      if (types[k] === want) {
        return true;
      }
    }
    return false;
  }

  /**
   * @param {string} distro
   * @param {string} pkg
   * @returns {string}
   */
  function docsPackageUrl(distro, pkg) {
    return (
      'https://docs.ros.org/en/' +
      encodeURIComponent(distro) +
      '/p/' +
      encodeURIComponent(pkg) +
      '/'
    );
  }

  /**
   * @param {HTMLElement} el
   * @param {Error} err
   */
  function showError(el, err) {
    el.classList.remove('related-packages--loading');
    el.classList.add('related-packages--error');
    el.innerHTML =
      '<p class="related-packages__status">Could not load package metadata. ' +
      'Rebuild the HTML documentation while online so the rosdistro cache is ' +
      'downloaded into <code>_static/rosdistro_cache/</code>, ' +
      'or check your network connection.</p>';
    if (typeof console !== 'undefined' && console.warn) {
      console.warn('related_packages:', err);
    }
  }

  /**
   * @param {HTMLElement} el
   * @param {Record<string, string>} xmls
   */
  function fillWidget(el, xmls) {
    var want = el.getAttribute('data-build-type') || '';
    var max = parseInt(el.getAttribute('data-max') || '10', 10);
    var distro = el.getAttribute('data-distro') || 'rolling';

    var names = Object.keys(xmls).filter(function (name) {
      var xmlStr = xmls[name];
      if (typeof xmlStr !== 'string') {
        return false;
      }
      return matchesBuildType(xmlStr, want);
    });
    names.sort(function (a, b) {
      return a.localeCompare(b);
    });
    var picked = names.slice(0, max);

    var ul = document.createElement('ul');
    ul.className = 'related-packages__list';
    var j;
    for (j = 0; j < picked.length; j += 1) {
      var pkg = picked[j];
      var li = document.createElement('li');
      var a = document.createElement('a');
      a.href = docsPackageUrl(distro, pkg);
      a.textContent = pkg;
      a.rel = 'noopener noreferrer';
      li.appendChild(a);
      ul.appendChild(li);
    }

    el.innerHTML = '';
    el.classList.remove('related-packages--loading');

    if (picked.length === 0) {
      var p = document.createElement('p');
      p.className = 'related-packages__empty';
      p.textContent = 'No packages matched this filter.';
      el.appendChild(p);
    } else {
      el.appendChild(ul);
    }
  }

  function fillAll() {
    var widgets = document.querySelectorAll('.js-related-packages');
    if (!widgets.length) {
      return;
    }

    /** @type {Record<string, HTMLElement[]>} */
    var byDistro = {};
    var idx;
    for (idx = 0; idx < widgets.length; idx += 1) {
      var el = widgets[idx];
      var d = el.getAttribute('data-distro') || 'rolling';
      if (!byDistro[d]) {
        byDistro[d] = [];
      }
      byDistro[d].push(el);
    }

    var distroKeys = Object.keys(byDistro);
    var di;
    for (di = 0; di < distroKeys.length; di += 1) {
      (function (distro) {
        var group = byDistro[distro];
        loadXmls(distro).then(
          function (xmls) {
            var gi;
            for (gi = 0; gi < group.length; gi += 1) {
              fillWidget(group[gi], xmls);
            }
          },
          function (err) {
            var ei;
            for (ei = 0; ei < group.length; ei += 1) {
              showError(group[ei], err);
            }
          }
        );
      })(distroKeys[di]);
    }
  }

  if (typeof document !== 'undefined') {
    if (document.readyState === 'loading') {
      document.addEventListener('DOMContentLoaded', fillAll);
    } else {
      fillAll();
    }
  }
})();
