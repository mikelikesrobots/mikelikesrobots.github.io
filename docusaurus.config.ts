import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Mike Likes Robots',
  tagline: 'Where we share knowledge to accelerate robotics.',
  favicon: 'favicon.ico',

  url: 'https://mikelikesrobots.github.io',
  baseUrl: '/',

  organizationName: 'mikelikesrobots', // Usually your GitHub org/user name.
  projectName: 'mikelikesrobots.github.io', // Usually your repo name.
  deploymentBranch: 'gh-pages',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

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
          blogSidebarCount: 'ALL'
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        gtag: {
          trackingID: "G-7X3VQWMTBV"
        }
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
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
        {
          href: 'https://www.youtube.com/@mikelikesrobots',
          'aria-label': 'YouTube',
          position: 'right',
          html: '<i class="fab fa-youtube fa-lg"></i>',
          className: 'header-link',
        },
        {
          href: 'https://github.com/mikelikesrobots',
          'aria-label': 'GitHub',
          html: '<i class="fab fa-github fa-lg"></i>',
          position: 'right',
          className: 'header-link',
        },
        {
          href: 'https://www.linkedin.com/in/michael-hart-a7614262/',
          'aria-label': 'LinkedIn',
          position: 'right',
          html: '<i class="fab fa-linkedin fa-lg"></i>',
          className: 'header-link',
        },
        {
          href: 'https://www.reddit.com/user/mikelikesrobots',
          'aria-label': 'Reddit',
          position: 'right',
          html: '<i class="fab fa-reddit fa-lg"></i>',
          className: 'header-link',
        },
        {
          href: 'https://bsky.app/profile/mikelikesrobots.bsky.social',
          'aria-label': 'Bluesky',
          position: 'right',
          html: '<i class="fab fa-bluesky fa-lg small-pad-right"></i>',
          className: 'header-link',
        },
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
            },
            {
              href: 'https://www.reddit.com/user/mikelikesrobots',
              label: 'Reddit',
            },
            {
              href: 'https://www.linkedin.com/in/michael-hart-a7614262/',
              label: 'LinkedIn',
            },
            {
              href: 'https://bsky.app/profile/mikelikesrobots.bsky.social',
              label: 'Bluesky',
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
              href: 'https://github.com/facebook/docusaurus',
            },
            {
              label: 'Business Enquiries',
              href: 'mailto:mikelikesrobots@outlook.com',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Mike Likes Robots. Built with Docusaurus.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
  scripts: [
    {
      src: '/js/fontawesome-all.min.js',
      defer: true,
    }
  ],
};

export default config;
