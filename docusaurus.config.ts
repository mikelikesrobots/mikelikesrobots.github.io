import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

import remarkMath from 'remark-math';
import rehypeKatex from 'rehype-katex';

const motto = 'Where we share knowledge to accelerate robotics.';

const config: Config = {
  title: 'Mike Likes Robots',
  tagline: motto,
  favicon: 'favicon.ico',

  url: 'https://mikelikesrobots.github.io',
  baseUrl: '/',

  organizationName: 'mikelikesrobots', // Usually your GitHub org/user name.
  projectName: 'mikelikesrobots.github.io', // Usually your repo name.
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',

  markdown: {
    hooks: {
      onBrokenMarkdownImages: 'throw',
      onBrokenMarkdownLinks: 'throw',
    }
  },


  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: ['./plugins/article-metadata'],

  themes: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        indexBlog: true,
        indexDocs: true,
        indexPages: true,
      },
    ],
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts'
        },
        blog: {
          showReadingTime: true,
          blogSidebarTitle: 'All posts',
          blogSidebarCount: 'ALL',
          remarkPlugins: [remarkMath],
          rehypePlugins: [rehypeKatex]
        },
        theme: {
          customCss: ['./src/css/custom.css', require.resolve('katex/dist/katex.min.css')],
        },
        gtag: {
          trackingID: "G-7X3VQWMTBV"
        }
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card-mascot.png',
    docs: {
      sidebar: {
        hideable: true,
      },
    },
    navbar: {
      title: 'Mike Likes Robots',
      logo: {
        alt: 'Ike-U - mascot icon for blog',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'gettingStartedSidebar',
          position: 'left',
          label: 'Getting Started',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {to: '/hireme', label: 'Hire Me', position: 'left'},
        {href: 'https://www.youtube.com/@mikelikesrobots', label: 'YouTube', position: 'right', className: 'social-link social-link--youtube'},
        {href: 'https://github.com/mikelikesrobots', label: 'GitHub', position: 'right', className: 'social-link social-link--github'},
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Docs',
          items: [
            {
              label: 'Getting Started Guide',
              to: '/docs/getting-started/intro',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              href: 'https://www.youtube.com/channel/UCAdis20vFM97syzwG1YlacQ',
              label: 'YouTube',
              className: 'social-link social-link--youtube',
            },
            {
              href: 'https://www.reddit.com/user/mikelikesrobots',
              label: 'Reddit',
              className: 'social-link social-link--reddit',
            },
            {
              href: 'https://www.linkedin.com/in/michael-hart-a7614262/',
              label: 'LinkedIn',
              className: 'social-link social-link--linkedin',
            },
            {
              href: 'https://bsky.app/profile/mikelikesrobots.bsky.social',
              label: 'Bluesky',
              className: 'social-link social-link--bluesky',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              className: 'social-link social-link--github',
              href: 'https://github.com/mikelikesrobots',
            },
            {
              label: 'Business Enquiries',
              href: 'mailto:mikelikesrobots@outlook.com',
            },
          ],
        },
      ],
      copyright: `<p class="footer-motto">${motto}</p><p>Copyright © ${new Date().getFullYear()} Mike Likes Robots. Built with Docusaurus.</p>`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
