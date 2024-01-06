# prettier-config
Code style definitions for Harker Robotics. 

## Installation
Install this package locally and [prettier](https://www.npmjs.com/package/prettier) globally.

    npm i -g prettier
    npm i --save-dev @harker-robo/prettier-config

## Usage
1. Add the `prettier` key in package.json
```jsonc
// package.json
{
    ...
    "prettier": "@harker-robo/prettier-config"
}
```

2. Run the formatter.
```
prettier ./* -w
```

## Links
* [Ignoring files](https://prettier.io/docs/en/ignore.html)
* [Overriding configuration](https://prettier.io/docs/en/configuration.html)
* [GitHub integration](https://github.com/marketplace/actions/prettier-action)
* [Editor integration](https://prettier.io/docs/en/editors.html)