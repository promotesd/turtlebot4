"""Interchangeable Q-network implementations."""

from .base import QNetwork
from .numpy_network import NumpyQNetwork

__all__ = ['NumpyQNetwork', 'QNetwork']
